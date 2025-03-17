#pragma once

// #include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

#include <mujoco_ros/common_types.hpp>
#include <mujoco_ros/mujoco_env.hpp>
// #include <mujoco_ros/logging.hpp>

// #include <mujoco_ros_msgs/RegisterSensorNoiseModels.h>
#include <mujoco_ros_msgs/srv/register_sensor_noise_models.hpp>
#include <mujoco_ros_msgs/msg/scalar_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
// #include <mujoco_ros/ros_one/plugin_utils.h>
#include <mujoco_ros/ros_two/plugin_utils.hpp>

#include <random>

namespace mujoco_ros::sensors {
struct SensorConfig
{
public:
	SensorConfig() : frame_id(""){};
	SensorConfig(std::string frame_id) : frame_id(std::move(frame_id)){};

	void setFrameId(const std::string &frame_id) { this->frame_id = frame_id; };

    template<typename M> rclcpp::SerializedMessage serializeMessage(const M &msg){
		rclcpp::Serialization<M> serializer;
		rclcpp::SerializedMessage serialized_msg;
		serializer.serialize_message(&msg, &serialized_msg);
		return serialized_msg;
	};
    std::string frame_id;

	// ros::Publisher gt_pub;
	// ros::Publisher value_pub;
	rclcpp::GenericPublisher::SharedPtr gt_pub;
	rclcpp::GenericPublisher::SharedPtr value_pub;

	// Noise params
	double mean[3];
	double sigma[3];

	uint8_t is_set = 0; // 0 for unset, otherwise binary code for combination of dims
};

struct rfConfig{
	std_msgs::msg::Float64MultiArray msg_;
	int rf_count_;
	std::vector<int> sensor_ids_;
	int body_id_;
	bool valid_ = false;
};
using rfConfigPtr = std::unique_ptr<rfConfig>;
using SensorConfigPtr = std::unique_ptr<SensorConfig>;

class MujocoRosRangeFinder : public mujoco_ros::MujocoPlugin
 {
    public:
	~MujocoRosRangeFinder() override;

	// Overload entry point
	bool Load(const mjModel *m, mjData *d) override; // load -> Load

	void Reset() override; // reset -> Reset

	void LastStageCallback(const mjModel *model, mjData *data) override; // last -> Last

	mujoco_ros::CallbackReturn on_configure(const rclcpp_lifecycle::State &/*previous_state*/){
        // RCLCPP_INFO_STREAM(get_my_logger(), "Configuring DummyRos2Plugin");

        declare_parameter_if_not_declared(
            this->get_node()->get_node_parameters_interface(),
            "test_name",
            rclcpp::ParameterValue("parallel_node_name")
        );
		if (!this->get_node()->has_parameter("rangefinder")) {
			this->get_node()->declare_parameter<std::vector<std::string>>("rangefinder", std::vector<std::string>());
		}


    return mujoco_ros::CallbackReturn::SUCCESS;

 }

 private:
	// replaced via env_ptr_
	rclcpp_lifecycle::LifecycleNode::SharedPtr sensors_nh_;


	void initSensors(const mjModel *model, mjData *data);
	std::mt19937 rand_generator = std::mt19937(std::random_device{}());
	std::normal_distribution<double> noise_dist;
    std::map<std::string, SensorConfigPtr> sensor_map_;
	std::map<std::string, rfConfigPtr> rangefinder_map_;

	void configurerf(const mjModel *model, mjData *data);
	void publishrf(const mjModel* model, mjData *data);

	
	static rclcpp::Logger getLogger(){
		return rclcpp::get_logger("MujocoRos2SensorPlugin");
    };

};

const char *SENSOR_STRING[37];

}