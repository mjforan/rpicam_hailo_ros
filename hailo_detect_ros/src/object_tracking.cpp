/**
 * Copyright (c) 2020-2025 Hailo Technologies Ltd. All rights reserved.
 * Distributed under the MIT license (https://opensource.org/licenses/MIT)
 * 2025 Modified by Matthew Foran to publish detections as ROS 2 messages.
 **/
/**
 * @file async_infer_basic_example.cpp
 * This example demonstrates the Async Infer API usage with a specific model.
 **/

#include "async_inference.hpp"
#include "utils.hpp"

#include "rclcpp/rclcpp.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "vision_msgs/msg/detection2_d.hpp"
#include "vision_msgs/msg/object_hypothesis_with_pose.hpp"

/////////// Constants ///////////
constexpr size_t MAX_QUEUE_SIZE = 10;
/////////////////////////////////

std::shared_ptr<BoundedTSQueue<PreprocessedFrameItem>> preprocessed_queue =
    std::make_shared<BoundedTSQueue<PreprocessedFrameItem>>(MAX_QUEUE_SIZE);

std::shared_ptr<BoundedTSQueue<InferenceOutputItem>>   results_queue =
    std::make_shared<BoundedTSQueue<InferenceOutputItem>>(MAX_QUEUE_SIZE);

void release_resources(cv::VideoCapture &capture) {
    capture.release();
    cv::destroyAllWindows();
    preprocessed_queue->stop();
    results_queue->stop();
}

hailo_status run_post_process(
    CommandLineArgs args,
    int input_w,
    int input_h,
    double pad_ratio_mult,
    std::shared_ptr<rclcpp::Node> ros_node,
    size_t class_count = 80)
    {
    auto ros_publisher = ros_node->create_publisher<vision_msgs::msg::Detection2DArray>("/hailo/detections", 10);
    while (true) {
        InferenceOutputItem output_item;
        if (!results_queue->pop(output_item)) {
            break;
        }
        auto bboxes = parse_nms_data(output_item.output_data_and_infos[0].first, class_count);
        // TODO update tracker and filter detections

        // Construct ROS message and publish
        vision_msgs::msg::Detection2DArray det_arr_msg;
        // TODO get timestamp of original image - not currently possible with Hailo tools
        det_arr_msg.header.stamp = ros_node->get_clock()->now();
        det_arr_msg.header.frame_id = "hailo_frame";
        for (const NamedBbox & detection : bboxes) {
            vision_msgs::msg::Detection2D det_msg;
            det_msg.header.stamp = det_arr_msg.header.stamp;
            det_msg.header.frame_id = det_arr_msg.header.frame_id;
            det_msg.id = "0"; // TODO get tracker ID
            det_msg.bbox.center.position.x = input_w*(detection.bbox.x_min + detection.bbox.x_max)/2;
            det_msg.bbox.center.position.y = input_h*pad_ratio_mult*(detection.bbox.y_min + detection.bbox.y_max)/2;
            det_msg.bbox.size_x            = input_w*(detection.bbox.x_max - detection.bbox.x_min);
            det_msg.bbox.size_y            = input_h*pad_ratio_mult*(detection.bbox.y_max - detection.bbox.y_min);
            det_msg.results.push_back(vision_msgs::msg::ObjectHypothesisWithPose());
            det_msg.results[0].hypothesis.class_id = std::to_string(detection.class_id);
            det_msg.results[0].hypothesis.score    = detection.bbox.score;
            det_arr_msg.detections.push_back(det_msg);
        }

        ros_publisher->publish(det_arr_msg);
    }
    return HAILO_SUCCESS;
}

hailo_status run_preprocess(CommandLineArgs args, AsyncModelInfer &model, 
                            cv::VideoCapture &capture, uint32_t target_w, uint32_t target_h,
                            uint32_t pad) {
    cv::Mat frame;
    cv::Mat resized;
    capture >> frame;
    while (true) {
        // Hailo AsyncInference will push as many frames as it receives to the device, so when
        // that buffer fills it will cause huge latency. As a simple workaround we can discard
        // 1/2 of the frames
        //capture.grab();
        capture >> frame;
        if (frame.empty()) {
            preprocessed_queue->stop();
            break;
        }        
        PreprocessedFrameItem preprocessed_frame_item;
        // Keep image aspect ratio when resizing, then pad with zeros to fit model input
        cv::resize(frame, resized, cv::Size(target_w, target_h));
        cv::copyMakeBorder(resized, preprocessed_frame_item.resized_for_infer, 0, pad, 0, 0, cv::BORDER_CONSTANT, cv::Scalar(0,0,0));
        preprocessed_queue->push(preprocessed_frame_item);
    }
    release_resources(capture);
    return HAILO_SUCCESS;
}

hailo_status run_inference_async(AsyncModelInfer& model,
                            std::chrono::duration<double>& inference_time) {
    
    auto start_time = std::chrono::high_resolution_clock::now();
    while (true) {
        PreprocessedFrameItem item;
        if (!preprocessed_queue->pop(item)) {
            break;
        }
        model.infer(std::make_shared<cv::Mat>(item.resized_for_infer), cv::Mat());
    }
    model.get_queue()->stop();
    auto end_time = std::chrono::high_resolution_clock::now();

    inference_time = end_time - start_time;

    return HAILO_SUCCESS;
}

int main(int argc, char** argv)
{
    CommandLineArgs args;

    rclcpp::init(argc, argv);
    auto ros_node = std::make_shared<rclcpp::Node>("hailo_detect");
    ros_node->declare_parameter("model", "/model.hef");
    args.detection_hef = ros_node->get_parameter("model").as_string();
    ros_node->declare_parameter("source", "/dev/video0");
    args.input_path = ros_node->get_parameter("source").as_string();

    size_t class_count = 80; // 80 classes in COCO dataset

    std::chrono::duration<double> inference_time;
    cv::VideoCapture capture;

    AsyncModelInfer model(args.detection_hef, results_queue);
    auto model_input_shape = model.get_infer_model()->hef().get_input_vstream_infos().release()[0].shape;
    uint32_t model_w = model_input_shape.width;
    uint32_t model_h = model_input_shape.height;

    capture.open(args.input_path, cv::CAP_ANY); 
    if (!capture.isOpened()) {
        throw std::runtime_error("Unable to read input file");
    }
    int input_w = capture.get(cv::CAP_PROP_FRAME_WIDTH);
    int input_h = capture.get(cv::CAP_PROP_FRAME_HEIGHT);
    double aspect_ratio_inv = (double)input_h / input_w;
    // Padding to keep resized image at same aspect ratio
    uint32_t pad = model_h - int(model_w * aspect_ratio_inv);
    // Account for padding when converting bbox to absolute pixel values
    double pad_ratio_mult = (double)model_h/(model_h-pad);

    auto preprocess_thread = std::async(run_preprocess,
                                        args,
                                        std::ref(model),
                                        std::ref(capture),
                                        model_w,
                                        model_w * aspect_ratio_inv,
                                        pad);

    auto inference_thread = std::async(run_inference_async,
                                    std::ref(model),
                                    std::ref(inference_time));

    auto output_parser_thread = std::async(run_post_process,
                                args,
                                input_w,
                                input_h,
                                pad_ratio_mult,
                                ros_node,
                                class_count);

    hailo_status status = wait_and_check_threads(
        preprocess_thread,    "Preprocess",
        inference_thread,     "Inference",
        output_parser_thread, "Postprocess "
    );

    rclcpp::shutdown();
    return status;
}
