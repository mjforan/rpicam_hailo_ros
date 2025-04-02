#!/usr/bin/env python3
"""
Example module for Hailo Detection + ByteTrack + Supervision.
https://github.com/hailo-ai/Hailo-Application-Code-Examples/blob/main/runtime/python/detection_with_tracker/detection_with_tracker.py
"""

import argparse
import supervision as sv
import numpy as np
import cv2
import queue
import sys
import os
from typing import Dict, List
import threading

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from utils import HailoAsyncInference


def initialize_arg_parser() -> argparse.ArgumentParser:
    """Initialize argument parser for the script."""
    parser = argparse.ArgumentParser(
        description="Detection Example - Tracker with ByteTrack and Supervision"
    )
    parser.add_argument(
        "-n", "--net", help="Path for the HEF model.", default="yolov5m_wo_spp_60p.hef"
    )
    parser.add_argument(
        "-i", "--input_video", default="input_video.mp4", help="Path to the input video."
    )
    parser.add_argument(
        "-l", "--labels", default="coco.txt", help="Path to a text file containing labels."
    )
    parser.add_argument(
        "-s", "--score_thresh", type=float, default=0.5, help="Score threshold - between 0 and 1."
    )
    return parser


def preprocess_frame(
    frame: np.ndarray, model_h: int, model_w: int, video_h: int, video_w: int, aspect_ratio_inv: float, pad: int
) -> np.ndarray:
    """Preprocess the frame to match the model's input size."""
    if model_h == video_h and model_w == video_w:
        return frame
    # Keep image aspect ratio when resizing, then pad with zeros to fit model input
    resized = cv2.resize(frame, (model_w, int(model_h * aspect_ratio_inv)))
    return cv2.copyMakeBorder(resized, 0, pad, 0, 0, cv2.BORDER_CONSTANT, value=[0,0,0])


def extract_detections(
    hailo_output: List[np.ndarray], h: int, w: int, pad_ratio_mult: float, threshold: float = 0.5
) -> Dict[str, np.ndarray]:
    """Extract detections from the HailoRT-postprocess output."""
    xyxy: List[np.ndarray] = []
    confidence: List[float] = []
    class_id: List[int] = []
    num_detections: int = 0

    for i, detections in enumerate(hailo_output):
        if len(detections) == 0:
            continue
        for detection in detections:
            bbox, score = detection[:4], detection[4]

            if score < threshold:
                continue

            # Convert bbox to xyxy absolute pixel values
            bbox[0], bbox[1], bbox[2], bbox[3] = (
                bbox[1] * w,
                bbox[0] * pad_ratio_mult * h,
                bbox[3] * w,
                bbox[2] * pad_ratio_mult * h,
            )

            xyxy.append(bbox)
            confidence.append(score)
            class_id.append(i)
            num_detections += 1

    return {
        "xyxy": np.array(xyxy),
        "confidence": np.array(confidence),
        "class_id": np.array(class_id),
        "num_detections": num_detections,
    }


def postprocess_detections(
    detections: Dict[str, np.ndarray],
    class_names: List[str],
    tracker: sv.ByteTrack,
) -> np.ndarray:
    """Postprocess the detections by annotating the frame with bounding boxes and labels."""
    if detections['xyxy'].size == 0:
        # Can't construct empty sv.Detections, use this dummy class for the right format
        class dotdict(dict):
            __getattr__ = dict.get
        return dotdict(detections)

    sv_detections = sv.Detections(
        xyxy=detections["xyxy"],
        confidence=detections["confidence"],
        class_id=detections["class_id"],
    )

    # Update detections with tracking information
    sv_detections = tracker.update_with_detections(sv_detections)
    return sv_detections


def publish_ros(ros_node: Node, detections: Dict[str, np.ndarray]):
    det_arr_msg = Detection2DArray()
    det_arr_msg.header.stamp = ros_node.get_clock().now().to_msg()
    det_arr_msg.header.frame_id = "hailo_frame"
    for i in range(detections.xyxy.shape[0]):
        det_msg = Detection2D()
        det_msg.header.stamp = det_arr_msg.header.stamp
        det_msg.header.frame_id = det_arr_msg.header.frame_id
        det_msg.id = str(detections.tracker_id[i])
        det_msg.bbox.center.position.x = (detections.xyxy[i][0] + detections.xyxy[i][2])/2
        det_msg.bbox.center.position.y = (detections.xyxy[i][1] + detections.xyxy[i][3])/2
        det_msg.bbox.size_x            = float(detections.xyxy[i][2] - detections.xyxy[i][0])
        det_msg.bbox.size_y            = float(detections.xyxy[i][3] - detections.xyxy[i][1])
        det_msg.results.append(ObjectHypothesisWithPose())
        det_msg.results[0].hypothesis.class_id = str(detections.class_id[i])
        det_msg.results[0].hypothesis.score    = float(detections.confidence[i])
        det_arr_msg.detections.append(det_msg)

    ros_node.publisher.publish(det_arr_msg)


def main() -> None:
    """Main function to run the video processing."""
    # Parse command-line arguments
    args = initialize_arg_parser().parse_args()

    input_queue: queue.Queue = queue.Queue()
    output_queue: queue.Queue = queue.Queue()

    hailo_inference = HailoAsyncInference(
        hef_path=args.net,
        input_queue=input_queue,
        output_queue=output_queue,
    )
    model_h, model_w, _ = hailo_inference.get_input_shape()

    # Initialize the ROS 2 publisher
    rclpy.init()
    ros_node = Node("hailo_tracker")
    ros_node.publisher = ros_node.create_publisher(Detection2DArray, '/hailo/detections', 10)

    # Initialize components for video processing
    cap = cv2.VideoCapture(args.input_video)
    if not cap.isOpened():
        ros_node.get_logger().fatal("Could not open video stream")
        exit(1)

    video_info = sv.VideoInfo.from_video_path(video_path=args.input_video)
    video_w, video_h = video_info.resolution_wh
    aspect_ratio_inv = video_h / video_w
    pad = model_h - int(model_h * aspect_ratio_inv) # Padding to keep resized image at same aspect ratio
    pad_ratio_mult = model_h/(model_h-pad) # Account for padding when converting bbox to absolute pixel values
    tracker = sv.ByteTrack()

    # Load class names from the labels file
    with open(args.labels, "r", encoding="utf-8") as f:
        class_names: List[str] = f.read().splitlines()

    # Start the asynchronous inference in a separate thread
    inference_thread: threading.Thread = threading.Thread(target=hailo_inference.run)
    inference_thread.start()

    # Process each frame in the video
    try:
        while True:
            # Hailo AsyncInference will push as many frames as it receives to the device, so when
            # that buffer fills it will cause huge latency. As a naive workaround we discard 2/3 of
            # the frames: 30fps -> 10fps
            cap.grab()
            cap.grab()
            ret, frame = cap.read()
            if not ret:
                ros_node.get_logger().error("End of stream or error reading frame.")
                continue

            # Preprocess the frame
            preprocessed_frame: np.ndarray = preprocess_frame(
                frame, model_h, model_w, video_h, video_w, aspect_ratio_inv, pad
            )

            # Put the frame into the input queue for inference
            input_queue.put([preprocessed_frame])

            # Get the inference result from the output queue
            results: List[np.ndarray]
            _, results = output_queue.get()

            # Extract detections from the inference results
            detections: Dict[str, np.ndarray] = extract_detections(
                results, video_h, video_w, pad_ratio_mult, args.score_thresh
            )

            # Filter detections with the tracker
            detections = postprocess_detections(
                detections, class_names, tracker
            )

            # Publish detections on ROS
            publish_ros(ros_node, detections)

    finally:
        cap.release()

        # Signal the inference thread to stop and wait for it to finish
        input_queue.put(None)
        inference_thread.join()


if __name__ == "__main__":
    main()