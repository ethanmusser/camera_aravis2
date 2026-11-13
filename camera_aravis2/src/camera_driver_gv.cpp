// Copyright (c) 2024 Fraunhofer IOSB and contributors
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Fraunhofer IOSB nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "camera_aravis2/camera_driver_gv.h"

// Std
#include <chrono>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <thread>

// ROS
#include <rcl_interfaces/msg/floating_point_range.hpp>
#include <rcl_interfaces/msg/integer_range.hpp>
#include <rclcpp/time.hpp>

// camera_aravis2
#include "camera_aravis2/conversion_utils.h"
#include "camera_aravis2/error.h"

namespace camera_aravis2
{

//==================================================================================================
CameraDriverGv::CameraDriverGv(const rclcpp::NodeOptions& options) :
  CameraDriver("camera_driver_gv", options)
{
    //--- setup parameters
    setupParameters();

    auto ptp_offset_desc        = rcl_interfaces::msg::ParameterDescriptor{};
    ptp_offset_desc.description = "Maximum absolute offset from the PTP master in nanoseconds when the camera exposes PtpOffsetFromMaster.";
    ptp_offset_desc.read_only   = true;
    ptp_max_offset_ns_          = declare_parameter<int64_t>("ptp_max_offset_ns", 1000000, ptp_offset_desc);
    if (ptp_max_offset_ns_ < 0)
        throw std::invalid_argument("ptp_max_offset_ns must be nonnegative.");

    is_verbose_enable_ = get_parameter("verbose").as_bool();

    //--- get parameter overrides, i.e. all parameters, including those that are not declared
    parameter_overrides_ = this->get_node_parameters_interface()->get_parameter_overrides();

    //--- open camera device
    CHECK_SUCCESS(discoverAndOpenCameraDevice(), logger_);

    //--- check if GEV Device.
    if (!arv_camera_is_gv_device(p_camera_))
    {
        RCLCPP_FATAL(logger_, "Camera is no GigE-Vision Device.");
        return;
    }

    //--- set up structs holding relevant information of camera streams
    CHECK_SUCCESS(setupCameraStreamStructs(), logger_);

    //--- set device control settings
    CHECK_SUCCESS(setDeviceControlSettings(), logger_);

    //--- set transport layer control settings
    p_gv_tl_control_ = std::make_shared<GvTransportLayerControl>();
    p_tl_control_    = std::dynamic_pointer_cast<GenTransportLayerControl>(p_gv_tl_control_);

    CHECK_SUCCESS(setTransportLayerControlSettings(), logger_);

    //--- set image format control settings
    CHECK_SUCCESS(setImageFormatControlSettings(), logger_);

    //--- set acquisition control settings
    CHECK_SUCCESS(setAcquisitionControlSettings(), logger_);

    //--- set analog control settings
    CHECK_SUCCESS(setAnalogControlSettings(), logger_);

    //--- initialize services
    CHECK_SUCCESS(initializeServices(), logger_);

    //--- load dynamic parameters
    setupDynamicParameters();

    //--- load diagnostics
    setupCameraDiagnosticPublisher();

    //--- check ptp
    if (p_gv_tl_control_ && p_gv_tl_control_->is_ptp_enable)
        checkPtpState();

    //--- print currently applied camera configuration
    printCameraConfiguration();

    //--- spawn camera stream in thread, so that initialization is not blocked
    is_spawning_         = true;
    spawn_stream_thread_ = std::thread(&CameraDriverGv::spawnCameraStreams, this);
}

//==================================================================================================
CameraDriverGv::~CameraDriverGv()
{
    // Guarded error object
    GuardedGError err;

    //--- stop acquisition
    if (p_device_)
    {
        arv_device_execute_command(p_device_, "AcquisitionStop", err.ref());
        CHECK_GERROR_MSG(err, logger_, "In executing 'AcquisitionStop'.");
    }

    //--- stop emitting signals for streams
    for (uint i = 0; i < streams_.size(); i++)
        if (streams_[i].p_arv_stream)
            arv_stream_set_emit_signals(streams_[i].p_arv_stream, FALSE);

    //--- join spawning thread
    is_spawning_ = false;
    if (spawn_stream_thread_.joinable())
        spawn_stream_thread_.join();

    //--- join diagnostic thread
    is_diagnostics_published_ = false;
    if (diagnostic_thread_.joinable())
    {
        diagnostic_thread_.join();
    }

    //--- join buffer threads
    for (uint i = 0; i < streams_.size(); i++)
    {
        Stream& stream = streams_[i];

        stream.is_buffer_processed = false;

        //--- push empty object into queue to do final loop
        stream.buffer_queue.push(std::make_pair(nullptr, nullptr));

        if (stream.buffer_processing_thread.joinable())
            stream.buffer_processing_thread.join();
    }

    //--- print stream statistics
    printStreamStatistics();

    //--- unref pointers
    for (uint i = 0; i < streams_.size(); i++)
    {
        if (streams_[i].p_arv_stream)
            g_object_unref(streams_[i].p_arv_stream);
    }

    //--- camera / device object is unreferences in parent class
}

//==================================================================================================
[[nodiscard]] bool CameraDriverGv::setTechSpecificTlControlSettings()
{
    GuardedGError err;

    std::string tmp_feature_name;
    rclcpp::ParameterValue tmp_param_value;
    std::vector<std::pair<std::string, rclcpp::ParameterValue>> tmp_param_values;

    bool is_parameter_set;

    //--- check if transport layer control is correctly initialized
    if (!p_gv_tl_control_)
    {
        RCLCPP_FATAL(logger_, "%s:  Pointer to GevTransportLayerControl is not initialized. ",
                     __PRETTY_FUNCTION__);

        return false;
    }

    //--- for specific parameters, proceed as follows:
    //---   1. Check and read parameter from launch parameters
    //---   2. If provided in launch parameter, set value on device
    //---   3. Get the set value from the device and store it in corresponding struct
    //---   4. If specific parameter was given in the launch parameters, check against the actual
    //---       value which has been read from the camera and issue a warning message if applicable

    //--- GevSCPSPacketSize
    tmp_feature_name = "GevSCPSPacketSize";
    RCLCPP_DEBUG(logger_, "Evaluating 'TransportLayerControl.%s'", tmp_feature_name.c_str());
    is_parameter_set = getTransportLayerControlParameter(tmp_feature_name, tmp_param_value);
    if (is_parameter_set)
        setFeatureValueFromParameter<int64_t>(tmp_feature_name, tmp_param_value);
    getFeatureValue<int64_t>(tmp_feature_name, p_gv_tl_control_->packet_size);
    if (is_parameter_set &&
        !isParameterValueEqualTo<int64_t>(tmp_param_value, p_gv_tl_control_->packet_size))
        config_warn_msgs_.push_back("'" + tmp_feature_name + "' is not as specified.");

    //--- GevSCPD
    tmp_feature_name = "GevSCPD";
    RCLCPP_DEBUG(logger_, "Evaluating 'TransportLayerControl.%s'", tmp_feature_name.c_str());
    is_parameter_set = getTransportLayerControlParameter(tmp_feature_name, tmp_param_value);
    if (is_parameter_set)
        setFeatureValueFromParameter<int64_t>(tmp_feature_name, tmp_param_value);
    getFeatureValue<int64_t>(tmp_feature_name, p_gv_tl_control_->inter_packet_delay);
    if (is_parameter_set &&
        !isParameterValueEqualTo<int64_t>(tmp_param_value, p_gv_tl_control_->inter_packet_delay))
        config_warn_msgs_.push_back("'" + tmp_feature_name + "' is not as specified.");

    //--- set PTP Enable
    tmp_feature_name = "PtpEnable";
    RCLCPP_DEBUG(logger_, "Evaluating 'TransportLayerControl.%s'", tmp_feature_name.c_str());
    is_parameter_set = getTransportLayerControlParameter(tmp_feature_name, tmp_param_value);
    if (is_parameter_set && !setFeatureValueFromParameter<bool>(tmp_feature_name, tmp_param_value))
        return false;
    getFeatureValue<bool>(tmp_feature_name, p_gv_tl_control_->is_ptp_enable);
    if (is_parameter_set &&
        !isParameterValueEqualTo<bool>(tmp_param_value, p_gv_tl_control_->is_ptp_enable))
        config_warn_msgs_.push_back("'" + tmp_feature_name + "' is not as specified.");

    return true;
}

//==================================================================================================
int CameraDriverGv::discoverNumberOfStreams()
{
    //--- upper bound for a plausible number of stream channels (see #62)
    constexpr int64_t MAX_NUM_STREAMS = 16;

    int64_t num_streams = 0;

    if (p_device_)
    {
        num_streams = arv_device_get_integer_feature_value(p_device_, "DeviceStreamChannelCount",
                                                           nullptr);

        if (num_streams == 0 && arv_camera_is_gv_device(p_camera_))
        {
            num_streams = arv_device_get_integer_feature_value(p_device_, "GevStreamChannelCount",
                                                               nullptr);
        }
    }

    if (num_streams == 0 || !p_device_)
    {
        num_streams = 1;
        RCLCPP_INFO(logger_, "Unable to automatically detect number of supported stream channels. "
                             "Setting num_streams = %i.",
                    static_cast<int>(num_streams));
    }
    else if (num_streams < 0 || num_streams > MAX_NUM_STREAMS)
    {
        RCLCPP_WARN(logger_, "Invalid number of stream channels reported by device: %s. "
                             "Setting num_streams = 1.",
                    std::to_string(num_streams).c_str());
        num_streams = 1;
    }
    else
    {
        RCLCPP_INFO(logger_, "Number of supported stream channels: %i",
                    static_cast<int>(num_streams));
    }

    return static_cast<int>(num_streams);
}

//==================================================================================================
void CameraDriverGv::tuneArvStream(ArvStream* p_stream) const
{
    if (!p_stream)
        return;

    if (!ARV_IS_GV_STREAM(p_stream))
    {
        RCLCPP_ERROR(logger_, "Stream is not a GV_STREAM");
        return;
    }

    gboolean b_auto_buffer               = FALSE;
    gboolean b_packet_resend             = TRUE;
    unsigned int timeout_packet          = 40;  // milliseconds
    unsigned int timeout_frame_retention = 200;

    if (b_auto_buffer)
        g_object_set(p_stream, "socket-buffer",
                     ARV_GV_STREAM_SOCKET_BUFFER_AUTO,
                     "socket-buffer-size", 0,
                     NULL);
    if (!b_packet_resend)
        g_object_set(p_stream, "packet-resend",
                     b_packet_resend
                       ? ARV_GV_STREAM_PACKET_RESEND_ALWAYS
                       : ARV_GV_STREAM_PACKET_RESEND_NEVER,
                     NULL);
    g_object_set(p_stream, "packet-timeout",
                 timeout_packet * 1000,
                 "frame-retention", timeout_frame_retention * 1000,
                 NULL);
}

//==================================================================================================
void CameraDriverGv::postFrameProcessingCallback(const uint stream_id)
{
    RCL_UNUSED(stream_id);
}

//==================================================================================================
bool CameraDriverGv::isTimestampReady(const uint stream_id, ArvBuffer* buffer)
{
    if (!p_gv_tl_control_ || !p_gv_tl_control_->is_ptp_enable)
        return true;

    const std::lock_guard<std::mutex> lock(ptp_mutex_);
    if (std::chrono::steady_clock::now() - last_ptp_check_ >= std::chrono::seconds(1))
        checkPtpState();
    if (!ptp_ready_)
    {
        RCLCPP_WARN_THROTTLE(logger_, *get_clock(), 5000, "Withholding images until PTP is synchronized. Status: %s", p_gv_tl_control_->ptp_status.c_str());
        return false;
    }

    const uint64_t timestamp = arv_buffer_get_timestamp(buffer);
    auto& last_timestamp     = streams_[stream_id].last_device_timestamp;
    if (timestamp == 0 || timestamp > static_cast<uint64_t>(std::numeric_limits<int64_t>::max()) || timestamp <= last_timestamp)
    {
        RCLCPP_WARN_THROTTLE(logger_, *get_clock(), 5000, "Dropping an invalid or nonincreasing PTP timestamp on stream %u.", stream_id);
        return false;
    }
    last_timestamp = timestamp;
    return true;
}

//==================================================================================================
void CameraDriverGv::checkPtpState()
{
    last_ptp_check_              = std::chrono::steady_clock::now();
    ptp_ready_                   = false;
    p_gv_tl_control_->ptp_status = "Unavailable";

    // Some cameras require a latch before reading their PTP snapshot.
    if (arv_device_get_feature(p_device_, "PtpDataSetLatch") != nullptr && !executeCommand("PtpDataSetLatch"))
        return;
    if (!getFeatureValue<std::string>("PtpStatus", p_gv_tl_control_->ptp_status) || p_gv_tl_control_->ptp_status != "Slave")
        return;

    if (arv_device_get_feature(p_device_, "PtpOffsetFromMaster") != nullptr)
    {
        if (!getFeatureValue<int64_t>("PtpOffsetFromMaster", p_gv_tl_control_->ptp_offset))
            return;
        if (p_gv_tl_control_->ptp_offset < -ptp_max_offset_ns_ || p_gv_tl_control_->ptp_offset > ptp_max_offset_ns_)
        {
            RCLCPP_WARN_THROTTLE(logger_, *get_clock(), 5000, "PTP offset %ld ns exceeds the %ld ns limit.", p_gv_tl_control_->ptp_offset, ptp_max_offset_ns_);
            return;
        }
    }
    ptp_ready_ = true;
}

}  // end namespace camera_aravis2

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(camera_aravis2::CameraDriverGv)
