#include <mujoco/mujoco.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

class MuJoCoRangefinderNode : public rclcpp::Node {
public:
    MuJoCoRangefinderNode() : Node("mujoco_rangefinder") {
        // Publisher for rangefinder sensor data
        publisher_ = this->create_publisher<std_msgs::msg::Float64>("rangefinder_distance", 10);

        // Load MuJoCo model
        model_ = mj_loadXML("/home/pedrofigueroa/ros2_ws/src/multipanda_ros2/franka_description/mujoco/franka/scene.xml", nullptr, error_msg_, sizeof(error_msg_));
        if (!model_) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load MuJoCo model: %s", error_msg_);
            rclcpp::shutdown();
            return;
        }
        data_ = mj_makeData(model_);

        // Timer to publish data at 10 Hz
        timer_ = this->create_wall_timer(std::chrono::milliseconds(10),
                                         std::bind(&MuJoCoRangefinderNode::update_sensor, this));
    }

    ~MuJoCoRangefinderNode() {
        mj_deleteData(data_);
        mj_deleteModel(model_);
    }

private:
    void update_sensor() {
        for (int i = 0; i < 10; i++)
        {
          mj_step(model_, data_);  /* code */
        }
          // Advance simulation

        // Get sensor data
        int sensor_id = mj_name2id(model_, mjOBJ_SENSOR, "sensor");
        if (sensor_id == -1) {
            RCLCPP_WARN(this->get_logger(), "Sensor not found!");
            return;
        }

        int sensor_adr = model_->sensor_adr[sensor_id];  // Address in data->sensordata
        double range_value = data_->sensordata[sensor_adr];

        // Publish sensor data
        auto msg = std_msgs::msg::Float64();
        msg.data = range_value;
        publisher_->publish(msg);

        RCLCPP_INFO(this->get_logger(), "Rangefinder Distance: %.2f meters", range_value);
    }

    mjModel* model_;
    mjData* data_;
    char error_msg_[1000];

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MuJoCoRangefinderNode>());
    rclcpp::shutdown();
    return 0;
}

