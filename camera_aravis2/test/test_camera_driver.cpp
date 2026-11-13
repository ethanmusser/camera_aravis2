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

#if __has_include(<catch2/catch_test_macros.hpp>)
#include <catch2/catch_test_macros.hpp>
#else
#include <catch2/catch.hpp>
#endif

#include <chrono>
#include <functional>
#include <memory>
#include <thread>

using namespace std::chrono_literals;

namespace
{

class CameraFixture
{
  public:
    CameraFixture() :
      server_(arv_gv_fake_camera_new("127.0.0.1", "DriverTest"), &g_object_unref),
      context_(std::make_shared<rclcpp::Context>())
    {
        REQUIRE(server_ != nullptr);
        REQUIRE(arv_gv_fake_camera_is_running(server_.get()));
        context_->init(0, nullptr);
        rclcpp::ExecutorOptions executor_options;
        executor_options.context = context_;
        executor_                = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(executor_options);
        observer_                = std::make_shared<rclcpp::Node>("observer", rclcpp::NodeOptions().context(context_));
        executor_->add_node(observer_);
    }

    ~CameraFixture()
    {
        if (camera_)
            executor_->remove_node(camera_);
        camera_.reset();
        context_->shutdown("Test complete.");
    }

    void subscribe()
    {
        subscription_ = observer_->create_subscription<sensor_msgs::msg::Image>(
          "/camera/image_raw", rclcpp::QoS(1).best_effort(),
          [this](sensor_msgs::msg::Image::ConstSharedPtr)
          { ++images_; });
    }

    void connect()
    {
        auto options = rclcpp::NodeOptions().context(context_);
        options.arguments({"--ros-args", "-r", "__node:=camera"});
        options.parameter_overrides({
          {"guid", "127.0.0.1"},
          {"camera_info_urls", std::vector<std::string>{TEST_CONFIG_DIR "/camera_info_example_gv.yaml"}},
          {"ImageFormatControl.PixelFormat", "Mono8"},
          {"ImageFormatControl.Width", 64},
          {"ImageFormatControl.Height", 48},
          {"AcquisitionControl.AcquisitionFrameRate", 25.0},
          {"TransportLayerControl.GevSCPSPacketSize", 1500},
          {"camera.image_raw.enable_pub_plugins", std::vector<std::string>{"image_transport/raw"}},
        });
        camera_ = std::make_shared<camera_aravis2::CameraDriverGv>(options);
        executor_->add_node(camera_);
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
        return arv_fake_camera_get_acquisition_status(arv_gv_fake_camera_get_fake_camera(server_.get()));
    }

    std::unique_ptr<ArvGvFakeCamera, decltype(&g_object_unref)> server_;
    std::shared_ptr<rclcpp::Context> context_;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    std::shared_ptr<rclcpp::Node> observer_;
    std::shared_ptr<camera_aravis2::CameraDriverGv> camera_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    unsigned int images_ = 0;
};

}  // namespace

TEST_CASE("Acquisition starts for a subscriber present before camera construction", "[camera]")
{
    CameraFixture fixture;
    fixture.subscribe();
    fixture.connect();
    REQUIRE(fixture.waitFor([&fixture]
                            { return fixture.images_ >= 10; }));
    CHECK(fixture.acquisitionStatus() != 0);
}

TEST_CASE("Acquisition follows subscribers connected after initialization", "[camera]")
{
    CameraFixture fixture;
    fixture.connect();
    REQUIRE(fixture.waitFor([&fixture]
                            { return fixture.camera_->isInitialized(); }));
#ifdef WITH_MATCHED_EVENTS
    CHECK(fixture.acquisitionStatus() == 0);
#endif
    fixture.subscribe();
    REQUIRE(fixture.waitFor([&fixture]
                            { return fixture.images_ >= 10; }));
#ifdef WITH_MATCHED_EVENTS
    fixture.subscription_.reset();
    REQUIRE(fixture.waitFor([&fixture]
                            { return fixture.acquisitionStatus() == 0; }));
#endif
}
