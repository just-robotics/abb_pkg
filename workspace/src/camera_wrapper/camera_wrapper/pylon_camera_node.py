#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2 as cv

from pypylon import pylon


class PylonCameraNode(Node):

    def __init__(self):
        super().__init__("pylon_camera")

        # ---------- Parameters ----------
        self.declare_parameter("frame_id", "camera_link")
        self.declare_parameter("camera_topic", "image_raw")
        self.declare_parameter("fps", 30)
        self.declare_parameter("width", 1280)
        self.declare_parameter("height", 1024)
        self.declare_parameter("exposure_time", -1.0)
        self.declare_parameter("auto_exposure", True)

        self.frame_id = self.get_parameter("frame_id").value
        topic = self.get_parameter("camera_topic").value
        self.fps = float(self.get_parameter("fps").value)
        width = int(self.get_parameter("width").value)
        height = int(self.get_parameter("height").value)
        exposure = float(self.get_parameter("exposure_time").value)
        auto_exp = bool(self.get_parameter("auto_exposure").value)

        # ---------- Publisher ----------
        self.publisher = self.create_publisher(Image, topic, 10)
        self.bridge = CvBridge()

        # ---------- Camera ----------
        self.get_logger().info("Connecting to Basler camera...")

        self.camera = pylon.InstantCamera(
            pylon.TlFactory.GetInstance().CreateFirstDevice()
        )

        self.camera.Open()

        # ---------- Reset ----------
        self.camera.UserSetSelector.Value = "Default"
        self.camera.UserSetLoad.Execute()

        # ---------- Free run ----------
        self.camera.TriggerMode.Value = "Off"
        self.camera.AcquisitionMode.Value = "Continuous"

        # ---------- Resolution ----------
        try:
            self.camera.Width.Value = min(width, self.camera.Width.Max)
            self.camera.Height.Value = min(height, self.camera.Height.Max)
        except Exception as e:
            self.get_logger().warn(f"Resolution not set: {e}")

        # ---------- Exposure ----------
        try:
            if auto_exp:
                self.camera.ExposureAuto.Value = "Continuous"
            else:
                self.camera.ExposureAuto.Value = "Off"
                if exposure > 0:
                    self.camera.ExposureTime.Value = exposure * 1000.0
        except Exception as e:
            self.get_logger().warn(f"Exposure setup failed: {e}")

        # ---------- FPS ----------
        try:
            self.camera.AcquisitionFrameRateEnable.Value = True
            self.camera.AcquisitionFrameRate.Value = self.fps
        except Exception:
            self.get_logger().warn("FPS control not supported by this camera")

        # ---------- Converter ----------
        self.converter = pylon.ImageFormatConverter()
        self.converter.OutputPixelFormat = pylon.PixelType_BGR8packed
        self.converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned

        # ---------- Start grabbing ----------
        self.camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)

        # ---------- Timer ----------
        self.timer = self.create_timer(
            1.0 / self.fps,
            self.grab_and_publish
        )

        self.clahe = cv.createCLAHE(
            clipLimit=1.0,
            tileGridSize=(8,8)
        )

        self.get_logger().info("Camera started successfully.")

    # ==========================================================
    # Grab + publish
    # ==========================================================

    def grab_and_publish(self):

        if not self.camera.IsGrabbing():
            return

        try:
            grab = self.camera.RetrieveResult(
                5000,
                pylon.TimeoutHandling_ThrowException
            )
        except Exception as e:
            self.get_logger().warn(f"Grab timeout: {e}")
            return

        if grab.GrabSucceeded():
            img = self.converter.Convert(grab)
            frame = img.GetArray()
            
            # ---------- Brightness normalization ----------
            lab = cv.cvtColor(frame, cv.COLOR_BGR2LAB)
            l, a, b = cv.split(lab)

            l = self.clahe.apply(l)

            lab = cv.merge((l, a, b))
            frame = cv.cvtColor(lab, cv.COLOR_LAB2BGR)

            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id

            self.publisher.publish(msg)

        grab.Release()

    # ==========================================================
    # Shutdown cleanly
    # ==========================================================

    def destroy_node(self):
        if self.camera.IsGrabbing():
            self.camera.StopGrabbing()

        if self.camera.IsOpen():
            self.camera.Close()

        super().destroy_node()


# ==============================================================
# Main
# ==============================================================

def main():
    rclpy.init()
    node = PylonCameraNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
