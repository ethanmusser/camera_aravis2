// Copyright (c) 2026 Ethan J. Musser
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
//    * Neither the name of the copyright holder nor the names of its
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

#if __has_include(<catch2/catch_test_macros.hpp>)
#include <catch2/catch_test_macros.hpp>
#else
#include <catch2/catch.hpp>
#endif

#include <chrono>
#include <filesystem>
#include <fstream>
#include <functional>
#include <future>
#include <limits>
#include <memory>
#include <thread>

using namespace std::chrono_literals;

namespace
{

class InspectableCamera : public camera_aravis2::CameraDriverGv
{
  public:
    using CameraAravisNodeBase::getFeatureValue;
    using CameraDriverGv::CameraDriverGv;

    bool acceptTimestamp(const uint64_t timestamp)
    {
        {
            const std::lock_guard<std::mutex> lock(ptp_mutex_);
            checkPtpState();
        }
        std::unique_ptr<ArvBuffer, decltype(&g_object_unref)> buffer(arv_buffer_new_allocate(1),
                                                                     &g_object_unref);
        arv_buffer_set_timestamp(buffer.get(), timestamp);
        return isTimestampReady(0, buffer.get());
    }
};

class CameraFixture
{
  public:
    explicit CameraFixture(bool with_ptp = false) :
      server_(nullptr, &g_object_unref),
      context_(std::make_shared<rclcpp::Context>())
    {
        if (with_ptp)
        {
            std::unique_ptr<ArvFakeCamera, decltype(&g_object_unref)> source(
              arv_fake_camera_new("XML"), &g_object_unref);
            size_t size      = 0;
            const char* data = arv_fake_camera_get_genicam_xml(source.get(), &size);
            std::string xml(data, size);
            xml.insert(xml.rfind("</RegisterDescription>"), R"(
    <Boolean Name="PtpEnable">
        <pValue>PtpEnableRegister</pValue>
        <OnValue>1</OnValue>
        <OffValue>0</OffValue>
    </Boolean>


    <IntReg Name="PtpEnableRegister">
        <Address>0x5010</Address>
        <Length>4</Length>
        <AccessMode>RW</AccessMode>
        <pPort>Device</pPort>
        <Sign>Unsigned</Sign>
        <Endianess>BigEndian</Endianess>
    </IntReg>


    <Enumeration Name="PtpStatus">
        <pIsAvailable>PtpStatusAvailable</pIsAvailable>
        <EnumEntry Name="Listening">
            <Value>0</Value>
        </EnumEntry>
        <EnumEntry Name="Slave">
            <Value>1</Value>
        </EnumEntry>
        <EnumEntry Name="Uncalibrated">
            <Value>2</Value>
        </EnumEntry>
        <pValue>PtpStatusRegister</pValue>
    </Enumeration>


    <IntReg Name="PtpStatusRegister">
        <Address>0x5000</Address>
        <Length>4</Length>
        <AccessMode>RO</AccessMode>
        <pPort>Device</pPort>
        <Sign>Unsigned</Sign>
        <Endianess>BigEndian</Endianess>
        <CachingMode>NoCache</CachingMode>
    </IntReg>


    <Boolean Name="PtpStatusAvailable">
        <pValue>PtpStatusAvailableRegister</pValue>
        <OnValue>1</OnValue>
        <OffValue>0</OffValue>
    </Boolean>


    <IntReg Name="PtpStatusAvailableRegister">
        <Address>0x5020</Address>
        <Length>4</Length>
        <AccessMode>RO</AccessMode>
        <pPort>Device</pPort>
        <Sign>Unsigned</Sign>
        <Endianess>BigEndian</Endianess>
        <CachingMode>NoCache</CachingMode>
    </IntReg>


    <IntReg Name="PtpOffsetFromMaster">
        <pIsAvailable>PtpOffsetAvailable</pIsAvailable>
        <Address>0x5008</Address>
        <Length>8</Length>
        <AccessMode>RO</AccessMode>
        <pPort>Device</pPort>
        <Sign>Signed</Sign>
        <Endianess>BigEndian</Endianess>
        <CachingMode>NoCache</CachingMode>
    </IntReg>


    <Boolean Name="PtpOffsetAvailable">
        <pValue>PtpOffsetAvailableRegister</pValue>
        <OnValue>1</OnValue>
        <OffValue>0</OffValue>
    </Boolean>


    <IntReg Name="PtpOffsetAvailableRegister">
        <Address>0x5024</Address>
        <Length>4</Length>
        <AccessMode>RO</AccessMode>
        <pPort>Device</pPort>
        <Sign>Unsigned</Sign>
        <Endianess>BigEndian</Endianess>
        <CachingMode>NoCache</CachingMode>
    </IntReg>


    <Command Name="PtpDataSetLatch">
        <pValue>PtpLatchRegister</pValue>
        <CommandValue>1</CommandValue>
    </Command>


    <IntReg Name="PtpLatchRegister">
        <Address>0x5014</Address>
        <Length>4</Length>
        <AccessMode>WO</AccessMode>
        <pPort>Device</pPort>
        <Sign>Unsigned</Sign>
        <Endianess>BigEndian</Endianess>
    </IntReg>
)");
            xml_path_ =
              std::filesystem::temp_directory_path() /
              ("aravis-ptp-" +
               std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) +
               ".xml");
            std::ofstream(xml_path_) << xml;
            server_.reset(
              arv_gv_fake_camera_new_full("127.0.0.1", "DriverTest", xml_path_.c_str()));
        }
        else
        {
            server_.reset(arv_gv_fake_camera_new("127.0.0.1", "DriverTest"));
        }
        REQUIRE(server_ != nullptr);
        REQUIRE(arv_gv_fake_camera_is_running(server_.get()));
        if (with_ptp)
        {
            writeRegister(0x5020, 1);
            writeRegister(0x5024, 1);
        }
        context_->init(0, nullptr);
        rclcpp::ExecutorOptions executor_options;
        executor_options.context = context_;
        executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(executor_options);
        observer_ =
          std::make_shared<rclcpp::Node>("observer", rclcpp::NodeOptions().context(context_));
        executor_->add_node(observer_);
    }

    ~CameraFixture()
    {
        if (camera_added_)
            executor_->remove_node(camera_);
        camera_.reset();
        context_->shutdown("Test complete.");
        server_.reset();
        if (!xml_path_.empty())
            std::filesystem::remove(xml_path_);
    }

    void subscribe()
    {
        subscription_ = observer_->create_subscription<sensor_msgs::msg::Image>(
          "/camera/image_raw", rclcpp::QoS(1).best_effort(),
          [this](sensor_msgs::msg::Image::ConstSharedPtr) { ++images_; });
    }

    void connect(const std::vector<rclcpp::Parameter>& extra_parameters = {})
    {
        auto options = rclcpp::NodeOptions().context(context_);
        options.arguments({"--ros-args", "-r", "__node:=camera"});
        std::vector<rclcpp::Parameter> parameters{
          {"guid", "127.0.0.1"},
          {"camera_info_urls",
           std::vector<std::string>{TEST_CONFIG_DIR "/camera_info_example_gv.yaml"}},
          {"ImageFormatControl.PixelFormat", "Mono8"},
          {"ImageFormatControl.Width", 64},
          {"ImageFormatControl.Height", 48},
          {"AcquisitionControl.AcquisitionFrameRate", 25.0},
          {"TransportLayerControl.GevSCPSPacketSize", 1500},
          {"camera.image_raw.enable_pub_plugins", std::vector<std::string>{"image_transport/raw"}},
        };
        parameters.insert(parameters.end(), extra_parameters.begin(), extra_parameters.end());
        options.parameter_overrides(parameters);
        camera_ = std::make_shared<InspectableCamera>(options);
        if (context_->is_valid())
        {
            executor_->add_node(camera_);
            camera_added_ = true;
        }
    }

    bool waitFor(const std::function<bool()>& ready, std::chrono::seconds timeout = 5s)
    {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while (!ready() && std::chrono::steady_clock::now() < deadline)
        {
            executor_->spin_some();
            std::this_thread::sleep_for(10ms);
        }
        return ready();
    }

    unsigned int acquisitionStatus() const
    {
        return arv_fake_camera_get_acquisition_status(
          arv_gv_fake_camera_get_fake_camera(server_.get()));
    }

    void writeRegister(guint32 address, guint32 value)
    {
        REQUIRE(arv_fake_camera_write_register(arv_gv_fake_camera_get_fake_camera(server_.get()),
                                               address, value));
    }

    void setPtpOffset(int64_t offset)
    {
        const guint64 encoded = GUINT64_TO_BE(static_cast<guint64>(offset));
        REQUIRE(arv_fake_camera_write_memory(arv_gv_fake_camera_get_fake_camera(server_.get()),
                                             0x5008, sizeof(encoded), &encoded));
    }

    void spinFor(std::chrono::milliseconds duration)
    {
        const auto deadline = std::chrono::steady_clock::now() + duration;
        while (std::chrono::steady_clock::now() < deadline)
        {
            executor_->spin_some();
            std::this_thread::sleep_for(10ms);
        }
    }

    std::filesystem::path xml_path_;
    std::unique_ptr<ArvGvFakeCamera, decltype(&g_object_unref)> server_;
    std::shared_ptr<rclcpp::Context> context_;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    std::shared_ptr<rclcpp::Node> observer_;
    std::shared_ptr<InspectableCamera> camera_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    unsigned int images_ = 0;
    bool camera_added_   = false;
};

}  // namespace

TEST_CASE("Acquisition starts for a subscriber present before camera construction", "[camera]")
{
    CameraFixture fixture;
    fixture.subscribe();
    fixture.connect();
    REQUIRE(fixture.waitFor([&fixture] { return fixture.images_ >= 10; }));
    REQUIRE(fixture.acquisitionStatus() != 0);
}

TEST_CASE("Acquisition follows subscribers connected after initialization", "[camera]")
{
    CameraFixture fixture;
    fixture.connect();
    REQUIRE(fixture.waitFor([&fixture] { return fixture.camera_->isInitialized(); }));
#ifdef WITH_MATCHED_EVENTS
    REQUIRE(fixture.acquisitionStatus() == 0);
#endif
    fixture.subscribe();
    REQUIRE(fixture.waitFor([&fixture] { return fixture.images_ >= 10; }));
#ifdef WITH_MATCHED_EVENTS
    fixture.subscription_.reset();
    REQUIRE(fixture.waitFor([&fixture] { return fixture.acquisitionStatus() == 0; }));
#endif
}

TEST_CASE("Rejected camera control batches leave the hardware unchanged", "[camera][parameters]")
{
    CameraFixture fixture;
    fixture.connect({{"dynamic_parameters_yaml_url", TEST_DATA_DIR "/dynamic_parameters.yaml"}});
    REQUIRE(fixture.camera_->set_parameter({"GainRaw", 1}).successful);
    const auto original_trigger = fixture.camera_->get_parameter("TriggerSource").as_string();

    const auto result =
      fixture.camera_->set_parameters_atomically({{"GainRaw", 2}, {"TriggerSource", "Invalid"}});
    REQUIRE_FALSE(result.successful);
    REQUIRE(fixture.camera_->get_parameter("GainRaw").as_int() == 1);
    REQUIRE(fixture.camera_->get_parameter("TriggerSource").as_string() == original_trigger);
    guint32 actual_gain = 0;
    REQUIRE(arv_fake_camera_read_register(arv_gv_fake_camera_get_fake_camera(fixture.server_.get()),
                                          ARV_FAKE_CAMERA_REGISTER_GAIN_RAW, &actual_gain));
    REQUIRE(actual_gain == 1);

    REQUIRE(fixture.camera_->set_parameter({"GainRaw", 3}).successful);
    REQUIRE(arv_fake_camera_read_register(arv_gv_fake_camera_get_fake_camera(fixture.server_.get()),
                                          ARV_FAKE_CAMERA_REGISTER_GAIN_RAW, &actual_gain));
    REQUIRE(actual_gain == 3);
}

TEST_CASE("PTP images wait for lock and stop after lock loss", "[camera][ptp]")
{
    CameraFixture fixture(true);
    fixture.subscribe();
    fixture.connect({{"TransportLayerControl.PtpEnable", true}, {"stream_timeout", 0.5}});
    fixture.spinFor(1200ms);
    REQUIRE(fixture.images_ == 0);
    REQUIRE(fixture.context_->is_valid());
    REQUIRE(fixture.acquisitionStatus() != 0);

    fixture.writeRegister(0x5000, 1);
    REQUIRE(fixture.waitFor([&fixture] { return fixture.images_ >= 10; }));
    fixture.writeRegister(0x5000, 2);
    fixture.spinFor(1200ms);
    const auto after_loss = fixture.images_;
    fixture.spinFor(200ms);
    REQUIRE(fixture.images_ == after_loss);

    fixture.writeRegister(0x5000, 1);
    REQUIRE(fixture.waitFor([&fixture, after_loss] { return fixture.images_ >= after_loss + 10; }));
}

TEST_CASE("PTP rejects offsets, failed reads, and backward timestamps", "[camera][ptp]")
{
    CameraFixture fixture(true);
    fixture.writeRegister(0x5000, 1);
    fixture.connect({{"TransportLayerControl.PtpEnable", true}});
    REQUIRE(fixture.waitFor([&fixture] { return fixture.camera_->isInitialized(); }));
    REQUIRE(fixture.camera_->acceptTimestamp(100));
    REQUIRE_FALSE(fixture.camera_->acceptTimestamp(100));
    REQUIRE_FALSE(fixture.camera_->acceptTimestamp(99));
    REQUIRE_FALSE(fixture.camera_->acceptTimestamp(0));
    REQUIRE_FALSE(fixture.camera_->acceptTimestamp(std::numeric_limits<uint64_t>::max()));
    REQUIRE(fixture.camera_->acceptTimestamp(101));

    fixture.setPtpOffset(1000001);
    REQUIRE_FALSE(fixture.camera_->acceptTimestamp(102));
    fixture.setPtpOffset(-1000001);
    REQUIRE_FALSE(fixture.camera_->acceptTimestamp(102));
    fixture.setPtpOffset(-1000000);
    REQUIRE(fixture.camera_->acceptTimestamp(102));

    fixture.writeRegister(0x5020, 0);
    REQUIRE_FALSE(fixture.camera_->acceptTimestamp(103));
    fixture.writeRegister(0x5020, 1);
    fixture.writeRegister(0x5024, 0);
    REQUIRE_FALSE(fixture.camera_->acceptTimestamp(103));
    fixture.writeRegister(0x5024, 1);
    REQUIRE(fixture.camera_->acceptTimestamp(103));
}

TEST_CASE("Discovery waits for a camera connected after startup", "[camera][recovery]")
{
    CameraFixture fixture;
    fixture.server_.reset();
    auto opening         = std::async(std::launch::async, [&fixture] { fixture.connect(); });
    const auto initially = opening.wait_for(2s);
    fixture.server_.reset(arv_gv_fake_camera_new("127.0.0.1", "DriverTest"));
    const auto connected = opening.wait_for(10s);
    if (connected != std::future_status::ready)
        fixture.context_->shutdown("Discovery test timed out.");
    opening.get();
    REQUIRE(initially == std::future_status::timeout);
    REQUIRE(connected == std::future_status::ready);
    fixture.subscribe();
    REQUIRE(fixture.waitFor([&fixture] { return fixture.images_ >= 10; }));
}

TEST_CASE("Shutdown interrupts discovery of an unavailable camera", "[camera][recovery]")
{
    CameraFixture fixture;
    fixture.server_.reset();
    auto opening         = std::async(std::launch::async, [&fixture] { fixture.connect(); });
    const auto initially = opening.wait_for(2s);
    fixture.context_->shutdown("Stopping camera discovery.");
    const auto stopped = opening.wait_for(6s);
    opening.get();
    REQUIRE(initially == std::future_status::timeout);
    REQUIRE(stopped == std::future_status::ready);
    REQUIRE_FALSE(fixture.camera_->isInitialized());
}

TEST_CASE("Stream timeout detects stalled reception with working camera control",
          "[camera][recovery]")
{
    CameraFixture fixture;
    fixture.connect({{"stream_timeout", 0.5}});
    REQUIRE(fixture.waitFor([&fixture] { return fixture.camera_->isInitialized(); }));
    fixture.spinFor(750ms);
    REQUIRE(fixture.context_->is_valid());
    fixture.subscribe();
    REQUIRE(fixture.waitFor([&fixture] { return fixture.images_ >= 10; }));
    fixture.writeRegister(ARV_FAKE_CAMERA_REGISTER_ACQUISITION, 0);
    int64_t width = 0;
    REQUIRE(fixture.camera_->getFeatureValue<int64_t>("Width", width));
    REQUIRE(width == 64);
    REQUIRE(fixture.waitFor([&fixture] { return !fixture.context_->is_valid(); }));
}
