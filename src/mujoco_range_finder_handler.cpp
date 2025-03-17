#include <mujoco_rangefinder/mujoco_range_finder_handler.h>

#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <mujoco_ros_msgs/msg/scalar_stamped.hpp>

// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <mujoco_ros/mujoco_env.hpp>

namespace mujoco_ros::sensors {

MujocoRosRangeFinder::~MujocoRosRangeFinder()
    {
        sensor_map_.clear();
        RCLCPP_DEBUG_STREAM(getLogger(), "Shutting down noise model server service");
        // register_noise_model_server_->shutdown();
    }

bool MujocoRosRangeFinder::Load(const mjModel *model, mjData *data)
	{
        RCLCPP_INFO(getLogger(), "Loading sensors plugin ...");
	if (env_ptr_->settings_.eval_mode) {
		RCLCPP_WARN(getLogger(), "Evaluation mode is active, ground truth topics won't be available!");
	} else {
		RCLCPP_WARN(getLogger(), "Train mode is active, ground truth topics will be available!");
	} 

    SENSOR_STRING[mjSENS_TOUCH]          = "touch";
	SENSOR_STRING[mjSENS_ACCELEROMETER]  = "accelerometer";
	SENSOR_STRING[mjSENS_VELOCIMETER]    = "velocimeter";
	SENSOR_STRING[mjSENS_GYRO]           = "gyro";
	SENSOR_STRING[mjSENS_FORCE]          = "force";
	SENSOR_STRING[mjSENS_TORQUE]         = "torque";
	SENSOR_STRING[mjSENS_MAGNETOMETER]   = "magnetometer";
	SENSOR_STRING[mjSENS_RANGEFINDER]    = "rangefinder";
	SENSOR_STRING[mjSENS_JOINTPOS]       = "jointpos";
	SENSOR_STRING[mjSENS_JOINTVEL]       = "jointvel";
	SENSOR_STRING[mjSENS_TENDONPOS]      = "tendonpos";
	SENSOR_STRING[mjSENS_TENDONVEL]      = "tendonvel";
	SENSOR_STRING[mjSENS_ACTUATORPOS]    = "actuatorpos";
	SENSOR_STRING[mjSENS_ACTUATORVEL]    = "actuatorvel";
	SENSOR_STRING[mjSENS_ACTUATORFRC]    = "actuatorfrc";
	SENSOR_STRING[mjSENS_BALLQUAT]       = "ballquat";
	SENSOR_STRING[mjSENS_BALLANGVEL]     = "ballangvel";
	SENSOR_STRING[mjSENS_JOINTACTFRC]    = "jointactfrc";
	SENSOR_STRING[mjSENS_JOINTLIMITPOS]  = "jointlimitpos";
	SENSOR_STRING[mjSENS_JOINTLIMITVEL]  = "jointlimitvel";
	SENSOR_STRING[mjSENS_JOINTLIMITFRC]  = "jointlimitfrc";
	SENSOR_STRING[mjSENS_TENDONLIMITPOS] = "tendonlimitpos";
	SENSOR_STRING[mjSENS_TENDONLIMITVEL] = "tendonlimitvel";
	SENSOR_STRING[mjSENS_TENDONLIMITFRC] = "tendonlimitfrc";
	SENSOR_STRING[mjSENS_FRAMEPOS]       = "framepos";
	SENSOR_STRING[mjSENS_FRAMEQUAT]      = "framequat";
	SENSOR_STRING[mjSENS_FRAMEXAXIS]     = "framexaxis";
	SENSOR_STRING[mjSENS_FRAMEYAXIS]     = "frameyaxis";
	SENSOR_STRING[mjSENS_FRAMEZAXIS]     = "framezaxis";
	SENSOR_STRING[mjSENS_FRAMELINVEL]    = "framelinvel";
	SENSOR_STRING[mjSENS_FRAMEANGVEL]    = "frameangvel";
	SENSOR_STRING[mjSENS_FRAMELINACC]    = "framelinacc";
	SENSOR_STRING[mjSENS_FRAMEANGACC]    = "frameangacc";
	SENSOR_STRING[mjSENS_SUBTREECOM]     = "subtreecom";
	SENSOR_STRING[mjSENS_SUBTREELINVEL]  = "subtreelinvel";
	SENSOR_STRING[mjSENS_SUBTREEANGMOM]  = "subtreeangmom";

    sensors_nh_ = get_node();
    configurerf(model, data);
	//initSensors(model, data);
	RCLCPP_INFO(sensors_nh_->get_logger(), "All sensors initialized");

    return true;

}

void MujocoRosRangeFinder::LastStageCallback(const mjModel *model, mjData *data)
{
	std::string sensor_name;

	int adr, type, noise_idx;
	mjtNum cutoff;
	double noise = 0.0;


	publishrf(model,data);

}

void MujocoRosRangeFinder::configurerf(const mjModel *model, mjData *data)
{
	auto rf_names = sensors_nh_->get_parameter("rangefinder").as_string_array();
	for (const auto &rf_name : rf_names) 
	{
		RCLCPP_INFO(getLogger(), "Configuring rangefinder %s", rf_name.c_str());
		sensors_nh_->declare_parameter<int>(rf_name+".rf_count",0);

		rfConfigPtr rfinder=std::make_unique<rfConfig>();
		if(sensors_nh_->has_parameter(rf_name + ".rf_count")){
			rfinder->rf_count_ = sensors_nh_->get_parameter(rf_name+".rf_count").as_int();
		}
		rfinder->valid_=(rfinder->rf_count_ > 0);
	
		std::string frame_id, site_name;
		int site_id;

		for(int i = 0; i < rfinder->rf_count_; i++)
		{
			site_name = rf_name + std::to_string(i);
			site_id = mj_name2id(const_cast<mjModel *>(model), mjOBJ_SITE, site_name.c_str());
			if(site_id == -1){
				rfinder->valid_ = false;
				break;
			}
			for(int n = 0; n < model->nsensor; n++)
			{
				if(model->sensor_objtype[n] == mjOBJ_SITE && model->sensor_objid[n] == site_id && model->sensor_type[n] == mjSENS_RANGEFINDER)
				{
					rfinder->sensor_ids_.push_back(n);
					break;
				}
			}


		}

		if(rfinder->sensor_ids_.size() != (size_t)rfinder->rf_count_){
			RCLCPP_WARN(getLogger(), "Number of rangefinder sensors in %s is not %d! Found %ld.", rf_name.c_str(), rfinder->rf_count_, rfinder->sensor_ids_.size());
			rfinder->valid_ = false;
		}

		if(!rfinder->valid_){
			RCLCPP_WARN(getLogger(), "Rangefinder %s is not valid! Skipping it.", rf_name.c_str());
			rangefinder_map_[rf_name] = std::move(rfinder); // still move it, to have record for what lidar names were given.
			continue;
		}
		rfinder->body_id_ = model->site_bodyid[site_id];
		frame_id = mj_id2name(const_cast<mjModel *>(model), mjOBJ_BODY, model->site_bodyid[site_id]);
		rfinder->msg_=std_msgs::msg::Float64MultiArray();

		RCLCPP_INFO(getLogger(), "Creating publishers for rangefinder %s in frame %s", rf_name.c_str(), frame_id.c_str());
		SensorConfigPtr config;
		config = std::make_unique<SensorConfig>(frame_id);
		config->value_pub = sensors_nh_->create_generic_publisher(rf_name, "std_msgs/msg/Float64MultiArray", 1);
		if (!env_ptr_->settings_.eval_mode) {
			config->gt_pub = sensors_nh_->create_generic_publisher(rf_name + "_GT", "std_msgs/msg/Float64MultiArray", 1);
		}
		sensor_map_[rf_name] = std::move(config);
		rangefinder_map_[rf_name] = std::move(rfinder);
	}


}

void MujocoRosRangeFinder::publishrf(const mjModel* model, mjData *data)
{
	for (const auto &rf_config : rangefinder_map_) 
	{
		if(!rf_config.second->valid_){
			continue;
		}

		for(int i = 0; i < rf_config.second->rf_count_; i++)
		{
			rf_config.second->msg_.data.push_back(data->sensordata[rf_config.second->sensor_ids_[i]]);

		}	

	sensor_map_[rf_config.first]->value_pub->publish(sensor_map_[rf_config.first]->serializeMessage(rf_config.second->msg_));
	
	rf_config.second->msg_.data.clear();

	}

}


void MujocoRosRangeFinder::Reset(){};

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mujoco_ros::sensors::MujocoRosRangeFinder, mujoco_ros::MujocoPlugin)

}