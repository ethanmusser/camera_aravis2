# Copyright (c) 2026 Ethan J. Musser
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


"""Exercise component reload after loss of a simulated GigE camera."""

import ctypes
from ctypes.util import find_library
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
import yaml


class FakeCamera:
    def __init__(self):
        self.library = ctypes.CDLL(find_library('aravis-0.8'))
        self.library.arv_gv_fake_camera_new.argtypes = [ctypes.c_char_p, ctypes.c_char_p]
        self.library.arv_gv_fake_camera_new.restype = ctypes.c_void_p
        self.library.arv_gv_fake_camera_is_running.argtypes = [ctypes.c_void_p]
        self.library.arv_gv_fake_camera_is_running.restype = ctypes.c_int
        self.library.g_object_unref.argtypes = [ctypes.c_void_p]
        self.library.g_object_unref.restype = None
        self.camera = None

    def start(self):
        self.camera = self.library.arv_gv_fake_camera_new(b'127.0.0.1', b'RecoveryTest')
        assert self.camera and self.library.arv_gv_fake_camera_is_running(self.camera)

    def stop(self):
        if self.camera:
            self.library.g_object_unref(self.camera)
            self.camera = None


def test_component_reload_after_camera_loss(tmp_path):
    calibration = Path(__file__).resolve().parents[1] / 'config/camera_info_example_gv.yaml'
    params = tmp_path / 'camera.yaml'
    params.write_text(yaml.safe_dump({'/**': {'ros__parameters': {
        'guid': '127.0.0.1',
        'camera_info_urls': [str(calibration)],
        'ImageFormatControl': {'PixelFormat': 'Mono8', 'Width': 64, 'Height': 48},
        'TransportLayerControl': {'GevSCPSPacketSize': 1500},
        'AcquisitionControl': {'AcquisitionFrameRate': 25.0},
        'camera.image_raw.enable_pub_plugins': ['image_transport/raw'],
    }}}))
    log_path = tmp_path / 'recovery.log'
    camera = FakeCamera()
    rclpy.init()
    node = rclpy.create_node('recovery_observer')
    images = []
    subscription = node.create_subscription(
        Image, '/camera/image_raw', lambda message: images.append(message.header.stamp),
        qos_profile_sensor_data,
    )

    def wait_for(ready, timeout):
        deadline = time.monotonic() + timeout
        while not ready() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        assert ready(), log_path.read_text()

    def retrying_after_loss():
        text = log_path.read_text()
        return ('Control to aravis device lost.' in text and
                'Camera unavailable; retrying discovery.' in
                text.split('Control to aravis device lost.')[-1])

    with log_path.open('w') as log:
        process = subprocess.Popen(
            ['ros2', 'launch', 'camera_aravis2', 'camera_driver_gv_composed.launch.py',
             'params_file:=' + str(params)],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
        )
        try:
            wait_for(lambda: 'Camera unavailable; retrying discovery.' in log_path.read_text(), 10)
            camera.start()
            wait_for(lambda: len(images) >= 10, 12)
            camera.stop()
            wait_for(retrying_after_loss, 25)
            # Keep the camera absent beyond the old fixed discovery retry count.
            deadline = time.monotonic() + 12
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
            before_return = len(images)
            camera.start()
            wait_for(lambda: len(images) >= before_return + 10, 15)
            assert process.poll() is None
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=25)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
            camera.stop()
            node.destroy_subscription(subscription)
            node.destroy_node()
            rclpy.shutdown()
    assert process.returncode == 0, log_path.read_text()
