import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst
import hailo
import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose

from hailo_apps_infra.hailo_rpi_common import app_callback_class
from hailo_apps_infra.detection_pipeline import GStreamerDetectionApp


class user_app_callback_class(app_callback_class, Node):
    def __init__(self):
        app_callback_class.__init__(self)
        Node.__init__(self, "hailo_detection_publisher")
        self.publisher_ = self.create_publisher(Detection2DArray, '/detections', 10)

    def publish(self, detections):
        det_arr_msg = Detection2DArray()
        det_arr_msg.header.stamp = self.get_clock().now()
        det_arr_msg.header.frame_id = "hailo_frame"

        for detection in detections:
            det_msg = Detection2D()
            result_msg = ObjectHypothesisWithPose()
            det_msg.header.stamp = det_arr_msg.header.stamp
            det_msg.header.frame_id = det_arr_msg.header.frame_id
            bbox = detection.get_bbox()
            det_msg.bbox.center.position.x = (bbox.xmin() + bbox.xmax())/2
            det_msg.bbox.center.position.y = (bbox.ymin() + bbox.ymax())/2
            det_msg.bbox.size_x            = bbox.width()
            det_msg.bbox.size_y            = bbox.height()
            result_msg.hypothesis.class_id = detection.get_label()
            result_msg.hypothesis.score    = detection.get_confidence()
            det_msg.results = [result_msg]
            det_arr_msg.append(det_msg)

        self.publisher_.publish(det_arr_msg)


def app_callback(_, info, ros_node):
    buffer = info.get_buffer()
    if buffer is None:
        return Gst.PadProbeReturn.OK
    roi = hailo.get_roi_from_buffer(buffer)
    detections = roi.get_objects_typed(hailo.HAILO_DETECTION)
    ros_node.publish(detections)
    return Gst.PadProbeReturn.OK

if __name__ == "__main__":
    rclpy.init()
    ros_node = user_app_callback_class()
    app = GStreamerDetectionApp(app_callback, ros_node)
    app.run()
