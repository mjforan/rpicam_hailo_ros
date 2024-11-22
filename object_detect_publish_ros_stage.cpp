#include "core/rpicam_app.hpp"
#include "post_processing_stages/post_processing_stage.hpp"
#include "object_detect.hpp"

#include <string>
#include <rclcpp/rclcpp.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection2_d.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>

using Rectange = libcamera::Rectangle;
using Stream = libcamera::Stream;

class ObjectDetectPublishRosStage : public PostProcessingStage
{
public:
	ObjectDetectPublishRosStage(RPiCamApp *app) : PostProcessingStage(app){
		rclcpp::init(0, nullptr);
	}

	char const *Name() const override;

	void Read(boost::property_tree::ptree const &params) override;

	void Configure() override;

	bool Process(CompletedRequestPtr &completed_request) override;

private:
	std::string detection_topic_;
	std::string frame_id_;
	rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr publisher_;
	rclcpp::Node::SharedPtr ros_node_;
};

#define NAME "object_detect_publish_ros"

char const *ObjectDetectPublishRosStage::Name() const
{
	return NAME;
}

void ObjectDetectPublishRosStage::Configure()
{
	ros_node_ = std::make_shared<rclcpp::Node>("picam_detect_" + frame_id_);
	publisher_ = ros_node_->create_publisher<vision_msgs::msg::Detection2DArray>(detection_topic_, 10);
}

void ObjectDetectPublishRosStage::Read(boost::property_tree::ptree const &params)
{
	detection_topic_ = params.get<std::string>("detection_topic", "detections");
	frame_id_        = params.get<std::string>("frame_id",        "pi_cam_0");
}

bool ObjectDetectPublishRosStage::Process(CompletedRequestPtr &completed_request)
{
	std::vector<Detection> detections;
	completed_request->post_process_metadata.Get("object_detect.results", detections);

	auto det_arr_msg = vision_msgs::msg::Detection2DArray();
	det_arr_msg.header.stamp = ros_node_->get_clock()->now();
	det_arr_msg.header.frame_id = frame_id_;
	
	for (auto &detection : detections)
	{
		auto det_msg = vision_msgs::msg::Detection2D();
		auto result_msg = vision_msgs::msg::ObjectHypothesisWithPose();
		det_msg.header.stamp = det_arr_msg.header.stamp;
		det_msg.header.frame_id = det_arr_msg.header.frame_id;
		det_msg.bbox.center.position.x = detection.box.x + detection.box.width/2;
		det_msg.bbox.center.position.y = detection.box.y + detection.box.height/2;
		det_msg.bbox.size_x            = detection.box.width;
		det_msg.bbox.size_y            = detection.box.height;
		result_msg.hypothesis.class_id = detection.name;
		result_msg.hypothesis.score    = detection.confidence;
		det_msg.results.push_back(result_msg);
		det_arr_msg.detections.push_back(det_msg);
	}
	publisher_->publish(det_arr_msg);

	return false;
}

static PostProcessingStage *Create(RPiCamApp *app)
{
	return new ObjectDetectPublishRosStage(app);
}

static RegisterStage reg(NAME, &Create);
