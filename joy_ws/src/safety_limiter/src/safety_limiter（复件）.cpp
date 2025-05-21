#include <rclcpp/rclcpp.hpp>
#include <math.h>
#include <array>
#include <memory>
#include <chrono>

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/header.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include "safety_limiter/msg/status.hpp"

#define MAX_XBOT_LIN_ACC_LIMIT 3.0
#define MAX_XBOT_ANG_ACC_LIMIT 3.0
#define MAX_XBOT_LIN_VEL_LIMIT 1.5
#define MAX_XBOT_ANG_VEL_LIMIT 3.0
#define ZERO_THRESHOLD 1e-3
#define WATCHDOG_INIT 10

using namespace std::chrono_literals;

template <typename T1, typename T2>
inline double dist2d(const T1 &_a, const T2 &_b)
{
	return hypot(_a.x - _b.x, _a.y - _b.y);
}

template <typename T>
inline double norm2d(const T &_a)
{
	return hypot(_a.x, _a.y);
}

template <typename T>
inline double norm2d(const T &_p1, const T &_p2)
{
	return hypot(_p1, _p2);
}

struct RobotState
{
	geometry_msgs::msg::Point pose;
	geometry_msgs::msg::Vector3 vel;
};

class Limiter : public rclcpp::Node
{
public:
	Limiter();
	~Limiter();

private:
	rclcpp::Time t_last_vel_update;

	std::string base_link_id;

	double lin_acc_;
	double max_acc_;
	double max_lin_margin_;
	double min_lin_margin_;
	double near_obs_margin_;
	double ang_acc_;
	double hz_;
	double max_lin_vel_;
	double max_ang_vel_;
	double max_scale_vel_;
	double t_window_;
	double robot_radius_;
	double deacc_margin_;
	double stop_deacc_;
	double cycle_interval_;

	int watchdog_cloud_;
	int watchdog_odom_;
	int watchdog_vel_;
	int dwa_steps_;

	bool disabled_;
	bool limit_ang_acc_;
	bool display_path_;
	bool display_radius_;
	bool visualize_vel_vector_;
	bool speed_limiter_mode_;

	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_vel_;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
	rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr sub_cloud_;
	rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_pose_;
	rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_disable_;
	rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_mode_toggle_;
	
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_vel_;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_obstacles_;
	rclcpp::Publisher<safety_limiter::msg::Status>::SharedPtr pub_status_;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_dwa_path_;
	rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr pub_margin_shape_;
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_nav_vel_vector_;
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_out_vel_vector_;
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_real_vel_vector_;

	rclcpp::TimerBase::SharedPtr timer_;
	std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
	std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

	nav_msgs::msg::Odometry odom_;
	nav_msgs::msg::Path dwa_path_;
	geometry_msgs::msg::PoseStamped path_pose_;
	geometry_msgs::msg::PoseStamped input_vel_vector_;
	geometry_msgs::msg::PoseStamped output_vel_vector_;
	sensor_msgs::msg::PointCloud cloud_;
	geometry_msgs::msg::Twist nav_cmd_;
	const geometry_msgs::msg::Twist zero_velocity_;

	void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
	void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
	void cloudCallback(const sensor_msgs::msg::PointCloud::SharedPtr msg);
	void poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
	void speedLimiterModeCallback(const std_msgs::msg::Bool::SharedPtr msg);
	void disableCallback(const std_msgs::msg::Bool::SharedPtr msg);
	void timerCallback();
	void performCollisionCheck();
	void checkParameters();
	void visualizeVelocityVectors(const geometry_msgs::msg::Twist &_out_vel) const;
	void publishCircularPolygon(const double _radius, const rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr &_publisher) const;
	inline double scaleLinearMargin(const double _curr_v) const;
	inline double projectVelocity(const double _curr_vel, const double _target_vel, const double _acc, const double _dt) const;

	geometry_msgs::msg::Twist limitOutputVelocity(const geometry_msgs::msg::Twist &_cmd_vel_in, const bool _is_slowing = false) const;
	sensor_msgs::msg::PointCloud filterPointcloud(const sensor_msgs::msg::PointCloud &_in_cloud, const geometry_msgs::msg::Point &_robot_pose, const double _radius) const;

	double robotDistanceToPoint(const geometry_msgs::msg::Point32 &_point, geometry_msgs::msg::Point _pos) const;
	double distanceToClosestObstacle(const sensor_msgs::msg::PointCloud &_col_obstacles, const geometry_msgs::msg::Point &_robot_pose) const;
};

Limiter::Limiter() : Node("safety_limiter"),
					 disabled_(false)
{
	// 声明和获取参数
	this->declare_parameter("cloud", "/cloud");
	this->declare_parameter("odom", "/odom");
	this->declare_parameter("cmd_vel", "cmd_vel");
	this->declare_parameter("cmd_vel_out", "/cmd_vel");
	this->declare_parameter("disable_command", "disable_command");
	this->declare_parameter("base_link_id", "base_link");
	this->declare_parameter("frequency_hz", 20.0);
	this->declare_parameter("terrain_condition", "");
	this->declare_parameter("max_lin_margin", 0.1);
	this->declare_parameter("min_lin_margin", 0.5);
	this->declare_parameter("near_obs_margin", 0.5);
	this->declare_parameter("max_acceleration_limit", 1.0);
	this->declare_parameter("max_angular_acceleration_limit", 1.0);
	this->declare_parameter("stop_acceleration", 2.0);
	this->declare_parameter("max_lin_vel", 1.0);
	this->declare_parameter("max_ang_vel", 1.2);
	this->declare_parameter("max_scale_vel", 1.0);
	this->declare_parameter("collision_time_window", 2.0);
	this->declare_parameter("dwa_steps", 20);
	this->declare_parameter("robot_radius", 0.3);
	this->declare_parameter("deacceleration_margin", 1.0/20.0);
	this->declare_parameter("limit_angular_vel", true);
	this->declare_parameter("display_path", false);
	this->declare_parameter("display_radius", false);
	this->declare_parameter("visualize_vel_vector", false);
	this->declare_parameter("speed_limiter_mode", false);
	this->declare_parameter("disabled_on_init", false);
	this->declare_parameter("limiter_mode_topic", "limiter_mode");

	std::string sCloud = this->get_parameter("cloud").as_string();
	std::string sOdom = this->get_parameter("odom").as_string();
	std::string sCmdVel = this->get_parameter("cmd_vel").as_string();
	std::string sCmdVelOut = this->get_parameter("cmd_vel_out").as_string();
	std::string sDisable = this->get_parameter("disable_command").as_string();
	std::string sTerrain = this->get_parameter("terrain_condition").as_string();
	std::string sSpeedLimiterMode = this->get_parameter("limiter_mode_topic").as_string();
	
	base_link_id = this->get_parameter("base_link_id").as_string();
	hz_ = this->get_parameter("frequency_hz").as_double();
	max_lin_margin_ = this->get_parameter("max_lin_margin").as_double();
	min_lin_margin_ = this->get_parameter("min_lin_margin").as_double();
	near_obs_margin_ = this->get_parameter("near_obs_margin").as_double();
	max_acc_ = this->get_parameter("max_acceleration_limit").as_double();
	ang_acc_ = this->get_parameter("max_angular_acceleration_limit").as_double();
	stop_deacc_ = this->get_parameter("stop_acceleration").as_double();
	max_lin_vel_ = this->get_parameter("max_lin_vel").as_double();
	max_ang_vel_ = this->get_parameter("max_ang_vel").as_double();
	max_scale_vel_ = this->get_parameter("max_scale_vel").as_double();
	t_window_ = this->get_parameter("collision_time_window").as_double();
	dwa_steps_ = this->get_parameter("dwa_steps").as_int();
	robot_radius_ = this->get_parameter("robot_radius").as_double();
	deacc_margin_ = this->get_parameter("deacceleration_margin").as_double();
	limit_ang_acc_ = this->get_parameter("limit_angular_vel").as_bool();
	display_path_ = this->get_parameter("display_path").as_bool();
	display_radius_ = this->get_parameter("display_radius").as_bool();
	visualize_vel_vector_ = this->get_parameter("visualize_vel_vector").as_bool();
	speed_limiter_mode_ = this->get_parameter("speed_limiter_mode").as_bool();
	disabled_ = this->get_parameter("disabled_on_init").as_bool();

	if (sTerrain == "marble")
		lin_acc_ = 1.0;
	else
		lin_acc_ = max_acc_;

	checkParameters();

	// 创建订阅者
	sub_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
		sCmdVel, 1, std::bind(&Limiter::cmdVelCallback, this, std::placeholders::_1));
	sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
		sOdom, 1, std::bind(&Limiter::odomCallback, this, std::placeholders::_1));
	sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud>(
		sCloud, 1, std::bind(&Limiter::cloudCallback, this, std::placeholders::_1));
	sub_disable_ = this->create_subscription<std_msgs::msg::Bool>(
		sDisable, 1, std::bind(&Limiter::disableCallback, this, std::placeholders::_1));
	sub_mode_toggle_ = this->create_subscription<std_msgs::msg::Bool>(
		sSpeedLimiterMode, 1, std::bind(&Limiter::speedLimiterModeCallback, this, std::placeholders::_1));

	// 创建发布者
	pub_vel_ = this->create_publisher<geometry_msgs::msg::Twist>(sCmdVelOut, 1);
	pub_obstacles_ = this->create_publisher<sensor_msgs::msg::PointCloud>("obstacles", 1);
	pub_status_ = this->create_publisher<safety_limiter::msg::Status>("status", 1);
	
	if (display_path_)
		pub_dwa_path_ = this->create_publisher<nav_msgs::msg::Path>("path", 1);
	if (display_radius_)
		pub_margin_shape_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>("margin", 1);
	if (visualize_vel_vector_)
	{
		pub_nav_vel_vector_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("nav_velocity_vector", 1);
		pub_real_vel_vector_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("real_velocity_vector", 1);
		pub_out_vel_vector_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("out_velocity_vector", 1);
	}

	// 初始化TF监听器
	tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
	tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

	// 创建定时器
	timer_ = this->create_wall_timer(
		std::chrono::duration<double>(1.0/hz_),
		std::bind(&Limiter::timerCallback, this));

	watchdog_cloud_ = watchdog_odom_ = watchdog_vel_ = -1;

	nav_cmd_ = zero_velocity_;

	if (speed_limiter_mode_)
	{
		auto status = std::make_unique<safety_limiter::msg::Status>();
		status->header.stamp = this->now();
		status->status = safety_limiter::msg::Status::SPEED_LIMITER_MODE;
		pub_status_->publish(std::move(status));
	}
	else if (disabled_)
	{
		auto status = std::make_unique<safety_limiter::msg::Status>();
		status->header.stamp = this->now();
		status->status = safety_limiter::msg::Status::DISABLED;
		pub_status_->publish(std::move(status));
	}
}

Limiter::~Limiter()
{
	geometry_msgs::msg::Twist cmd_vel;
	cmd_vel.linear.x = 0;
	cmd_vel.linear.y = 0;
	cmd_vel.angular.z = 0;
	pub_vel_->publish(cmd_vel);
}

void Limiter::cloudCallback(const sensor_msgs::msg::PointCloud::SharedPtr msg)
{
	cloud_ = *msg;
	watchdog_cloud_ = WATCHDOG_INIT;
	// ROS_INFO("Pointcloud received (%lu)", cloud_.points.size());
}

void Limiter::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
	odom_ = *msg;

	watchdog_odom_ = WATCHDOG_INIT;
}

void Limiter::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
	t_last_vel_update = this->now();
	nav_cmd_ = *msg;
	watchdog_vel_ = WATCHDOG_INIT;
}

void Limiter::disableCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
	disabled_ = msg->data;
	if (disabled_)
	{
		auto status = std::make_unique<safety_limiter::msg::Status>();
		status->header.stamp = this->now();
		status->status = safety_limiter::msg::Status::DISABLED;
		pub_status_->publish(std::move(status));
	}
}

void Limiter::speedLimiterModeCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
	speed_limiter_mode_ = msg->data;
	if (speed_limiter_mode_)
	{
		auto status = std::make_unique<safety_limiter::msg::Status>();
		status->header.stamp = this->now();
		status->status = safety_limiter::msg::Status::SPEED_LIMITER_MODE;
		pub_status_->publish(std::move(status));
	}
}

// check if parameters are valid and inside boundries
void Limiter::checkParameters()
{
	if (hz_ <= 0.0)
	{
		RCLCPP_WARN(this->get_logger(), "Frequency should be larger then zero. Setting to default value (%.1lf)", 20.0);
		hz_ = 20.0;
	}

	if (lin_acc_ <= 0.0 || lin_acc_ > MAX_XBOT_LIN_ACC_LIMIT)
	{
		RCLCPP_WARN(this->get_logger(), "Linear acceleration should be in range < 0.0, %.1lf ] (in ms-2). Setting to default value (%.2lf ms-2)", MAX_XBOT_LIN_ACC_LIMIT, 1.0);
		lin_acc_ = 1.0;
	}

	if (stop_deacc_ <= 0 || stop_deacc_ > MAX_XBOT_LIN_ACC_LIMIT)
	{
		RCLCPP_WARN(this->get_logger(), "Stop deacceleration should be in range < 0.0, %.1lf ] (in ms-2). Setting to default value (%.1lf)", MAX_XBOT_LIN_ACC_LIMIT, 2.0);
		stop_deacc_ = 2.0;
	}

	if (ang_acc_ <= 0 || ang_acc_ > MAX_XBOT_ANG_ACC_LIMIT)
	{
		RCLCPP_WARN(this->get_logger(), "Angular acceleration should be in range < 0.0, %.1lf ] (in rads/s2). Setting to default value (%.1lf)", MAX_XBOT_ANG_ACC_LIMIT, 1.0);
		ang_acc_ = 1.0;
	}

	if (dwa_steps_ <= 0)
	{
		RCLCPP_WARN(this->get_logger(), "Number of steps should be positive and larger then zero for collision detection to have effect. Setting to default value (%d)", 20);
		dwa_steps_ = 20;
	}

	if (t_window_ <= 0)
	{
		RCLCPP_WARN(this->get_logger(), "DWA time window should be positive and larger then zero for collision detection to have effect. Setting to default value (%.1lf)", 2.0);
		t_window_ = 2.0;
	}

	if (robot_radius_ <= 0)
	{
		RCLCPP_WARN(this->get_logger(), "Robot radius should be positive and larger then zero. Setting to default value (%.1lf)", 0.3);
		robot_radius_ = 0.3;
	}

	if (max_lin_vel_ <= 0 || max_lin_vel_ > MAX_XBOT_LIN_VEL_LIMIT)
	{
		RCLCPP_WARN(this->get_logger(), "Maximum linear velocity should be in range < 0.0, %.1lf ] (in ms). Setting to default value (%.1lf)", MAX_XBOT_LIN_VEL_LIMIT, 1.0);
		max_lin_vel_ = 1.0;
	}
	if (max_ang_vel_ <= 0 || max_ang_vel_ > MAX_XBOT_ANG_VEL_LIMIT)
	{
		RCLCPP_WARN(this->get_logger(), "Maximum angular velocity should be in range < 0.0, %.1lf ] (in ms). Setting to default value (%.1lf)", MAX_XBOT_ANG_VEL_LIMIT, 1.2);
		max_ang_vel_ = 1.0;
	}

	if (max_lin_margin_ < 0)
	{
		RCLCPP_WARN(this->get_logger(), "Maximum linear margin should be larger then zero. Setting to default value (%.1lf)", 0.1);
		max_lin_margin_ = 0.1;
	}

	if (min_lin_margin_ < 0 || min_lin_margin_ > max_lin_margin_)
	{
		RCLCPP_WARN(this->get_logger(), "Minimum linear margin should be in range < 0.0, %.1lf ] (in m), less then max_lin_margin_. Setting to default value (%.1lf)", max_lin_margin_, max_lin_margin_);
		min_lin_margin_ = max_lin_margin_;
	}

	if (near_obs_margin_ < 0)
	{
		RCLCPP_WARN(this->get_logger(), "Near obstacle margin should be positive (zero included). Setting to default value minimum linear margin (%.1lf)", min_lin_margin_);
		near_obs_margin_ = min_lin_margin_;
	}
}

// reduced pointcloud could be used to prevent near obstacle passes (side hits)
sensor_msgs::msg::PointCloud Limiter::filterPointcloud(const sensor_msgs::msg::PointCloud &_in_cloud, const geometry_msgs::msg::Point &_robot_pose, const double _filter_radius) const
{
	sensor_msgs::msg::PointCloud filtered_pc;
	filtered_pc.header = _in_cloud.header;
	filtered_pc.channels = _in_cloud.channels;

	for (const auto &point : _in_cloud.points)
	{
		if (dist2d(_robot_pose, point) <= _filter_radius)
			filtered_pc.points.emplace_back(point);
	}
	return filtered_pc;
}

// Calculate distance from a point (obstacle) and both robot center and offset center (caused by velodyne position)
// Returns smaller of two distances
double Limiter::robotDistanceToPoint(const geometry_msgs::msg::Point32 &_point, geometry_msgs::msg::Point _pos) const
{
	const double d1 = dist2d(_point, _pos);
	_pos.x -= 0.035;
	const double d2 = dist2d(_point, _pos);
	return d1 < d2 ? d1 : d2;
}

// Return distance from robot to closest obstacle taking into account two radius design
double Limiter::distanceToClosestObstacle(const sensor_msgs::msg::PointCloud &_col_obstacles, const geometry_msgs::msg::Point &_robot_pose) const
{
	double d_tmp, d_min = DBL_MAX;

	for (auto &point : _col_obstacles.points)
	{
		if ((d_tmp = robotDistanceToPoint(point, _robot_pose)) < d_min)
		{
			d_min = d_tmp;
		}
	}
	return d_min;
}

// Create and publish circular polygon from given radius (either being robot radius, margin or other), should only be used in debug mode
void Limiter::publishCircularPolygon(const double _radius, const rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr &_publisher) const
{
	static const std::array<double, 16> angles_rad{0, 0.392699, 0.785398, 1.1781, 1.5708, 1.9635, 2.35619, 2.74889, 3.14159, 3.53429, 3.92699, 4.31969, 4.71239, 5.10509, 5.49779, 5.89049}; // magic array, description below
	/*	Function used to generate angles array is below, constexpr lambda is not allowed in c++11 and constexpr functions are allowed but can only contain return which in this case is not sufficient

	std::array<double, 16> angles;
	constexpr const double single_angle = M_PI / 16.0;

	angles[0] = 0.0;

	for ( int i = 1; i<16 ; ++i)
	{
		angles[i] = angles[i-1] + single_angle;
	}
 	*/
	static auto genPoly = [](const double &_r, const std_msgs::msg::Header &_header) {
		geometry_msgs::msg::PolygonStamped poly;
		poly.header = _header;
		geometry_msgs::msg::Point32 point;

		for (int i = 0; i < 16; ++i)
		{
			point.x = _r * cos(angles_rad[i]);
			point.y = _r * sin(angles_rad[i]);
			poly.polygon.points.emplace_back(point);
		}
		return poly;
	};

	std_msgs::msg::Header header;
	header.frame_id = base_link_id;
	header.stamp = this->now();
	_publisher->publish(genPoly(_radius, header));
}

// calculate next velocity from current and target velocity considering velocity and acceleration limits
inline double Limiter::projectVelocity(const double _curr_vel, const double _target_vel, const double _acc, const double _dt) const
{
	if (_curr_vel < _target_vel)
	{
		const double v_next = _curr_vel + _acc * _dt;
		return std::min(_target_vel, v_next);
	}
	else
	{
		const double v_next = _curr_vel - _acc * _dt;
		return std::max(_target_vel, v_next);
	}
}

// calculate and return linear margin value for given velocity using margin scale method
inline double Limiter::scaleLinearMargin(const double _curr_v) const
{
	if (_curr_v > max_scale_vel_)
		return min_lin_margin_;
	if (_curr_v <= ZERO_THRESHOLD)
		return max_lin_margin_;

	return (min_lin_margin_ - max_lin_margin_) / max_scale_vel_ * _curr_v + max_lin_margin_;
}

// perform dwa-based collision check with obstacles, adjust approaching speed and publish new speed
void Limiter::performCollisionCheck()
{
	// geometry_msgs::msg::Twist *approx_cmd; // 注释掉未使用的变量
	geometry_msgs::msg::Twist out_cmd = nav_cmd_;
	sensor_msgs::msg::PointCloud col_obstacles;
	rclcpp::Time check_start_time{this->now()};

	bool skip_dwa = false;
	bool emergency_stop = false;
	bool is_slowing = false;

	col_obstacles.header.stamp = check_start_time;
	col_obstacles.header.frame_id = base_link_id;

	const double odom_v_norm{norm2d(odom_.twist.twist.linear)}, nav_v_norm{norm2d(nav_cmd_.linear)};
	const double scaled_lin_margin = scaleLinearMargin(odom_v_norm);

	if (display_radius_)
	{
		publishCircularPolygon(scaled_lin_margin + robot_radius_, pub_margin_shape_);
	}

	// no need to check for collisions if robot has no movement commands and it is stopped. Rotation in place will still be processed but othervise check will exit (and publish zero speed to be sure)
	if (norm2d(nav_cmd_.linear) <= ZERO_THRESHOLD && odom_v_norm <= ZERO_THRESHOLD)
	{
		if (fabs(nav_cmd_.angular.z) >= ZERO_THRESHOLD || fabs(odom_.twist.twist.angular.z) >= ZERO_THRESHOLD)
		{
			RCLCPP_DEBUG(this->get_logger(), "DWA skipped, only rotational velocity is being applied");
			skip_dwa = true;
		}
		else
		{
			pub_vel_->publish(zero_velocity_);
			return;
		}
	}

	if (!skip_dwa)
	{
		RobotState prevRobotState;
		geometry_msgs::msg::Pose curr_pose;
		sensor_msgs::msg::PointCloud filtered_cloud;

		if (display_path_)
		{
			dwa_path_.header = cloud_.header;
			path_pose_.header = cloud_.header;
		}

		const double cloud_radius = (odom_v_norm > nav_v_norm ? odom_v_norm : nav_v_norm) * t_window_ + robot_radius_ + max_lin_margin_;
		// transform pointcloud to base_link frame if needed
		if (base_link_id != cloud_.header.frame_id)
		{
			try
			{
				sensor_msgs::msg::PointCloud tf_cloud;
				tf_cloud.header = cloud_.header;
				tf_cloud.header.frame_id = base_link_id;

				//tf_buffer_->waitForTransform(base_link_id, cloud_.header.frame_id, cloud_.header.stamp, rclcpp::Duration(0.05s));
				//geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
				//	base_link_id, cloud_.header.frame_id, cloud_.header.stamp);
				//tf2::doTransform(cloud_, tf_cloud, transform);

				//filtered_cloud = filterPointcloud(tf_cloud, curr_pose.position, cloud_radius);

				//col_obstacles.header = filtered_cloud.header;
			}
			catch (const tf2::TransformException &e)
			{
				RCLCPP_ERROR(this->get_logger(), "%s, %d --- TF Exception\n%s", __FILE__, __LINE__, e.what());
				return;
			}
		}
		else
		{
			filtered_cloud = filterPointcloud(cloud_, curr_pose.position, cloud_radius);
		}

		// collision detection varaiables
		bool col_detected = false;

		// near obstacle detection variables
		geometry_msgs::msg::Point32 near_obstacle;
		double min_dist_obs = -1;
		double dist_step;
		bool near_obs;

		const double t_step = t_window_ / dwa_steps_;

		geometry_msgs::msg::Vector3 curr_vel;
		curr_vel.x = odom_.twist.twist.linear.x;
		curr_vel.y = odom_.twist.twist.linear.y;
		curr_vel.z = limit_ang_acc_ ? odom_.twist.twist.angular.z : nav_cmd_.angular.z;

		// DWA-based obstacle detection
		int col_step = 0;
		for (int step = 0; step <= dwa_steps_; step++)
		{
			near_obs = false;
			dist_step = DBL_MAX;

			for (const auto &point : filtered_cloud.points)
			{
				const double tmp_dist = robotDistanceToPoint(point, curr_pose.position);

				// rearranged previous implementation to match BVH model and possibly reduce total computation time
				if (tmp_dist <= (robot_radius_ + near_obs_margin_))
				{
					if (tmp_dist <= robot_radius_)
					{
						col_obstacles.points.emplace_back(point);
						col_detected = true;
					}
					else
					{
						near_obs = true;
						if (tmp_dist < dist_step)
						{
							near_obstacle = point;
							dist_step = tmp_dist;
						}
					}
				}
			}

			if (near_obs)
			{
				if (min_dist_obs > dist_step)
				{
					col_obstacles.points.emplace_back(near_obstacle);
					col_detected = true;
				}
				else
					min_dist_obs = dist_step;
			}

			if (col_detected)
			{
				col_step = step;
				break;
			}

			prevRobotState.pose = curr_pose.position;
			prevRobotState.vel = curr_vel;

			// adjust velocity used to approximate robots position
			curr_vel.x = projectVelocity(curr_vel.x, nav_cmd_.linear.x, lin_acc_, t_step);
			curr_vel.y = projectVelocity(curr_vel.y, nav_cmd_.linear.y, lin_acc_, t_step);
			if (limit_ang_acc_)
				curr_vel.z = projectVelocity(curr_vel.z, nav_cmd_.angular.z, ang_acc_, t_step);

			const double x_step_dist = curr_vel.x * t_step;
			const double y_step_dist = curr_vel.y * t_step;
			const double angle_step = curr_vel.z * t_step;

			curr_pose.orientation.z += angle_step;
			curr_pose.position.x += x_step_dist * cos(curr_pose.orientation.z) - y_step_dist * sin(curr_pose.orientation.z);
			curr_pose.position.y += y_step_dist * cos(curr_pose.orientation.z) + x_step_dist * sin(curr_pose.orientation.z);

			if (display_path_)
			{
				path_pose_.pose = curr_pose;
				dwa_path_.poses.emplace_back(path_pose_);
			}
		}

		if (col_detected)
		{
			const double col_step_lin_margin = scaleLinearMargin(norm2d(prevRobotState.vel));
			const double col_dist_margin = distanceToClosestObstacle(col_obstacles, prevRobotState.pose) - robot_radius_ - col_step_lin_margin;
			const int step_num = std::max(col_step - 1, 0);
			const double t_col = step_num * t_step + col_dist_margin / norm2d(prevRobotState.vel.x + ZERO_THRESHOLD, prevRobotState.vel.y + ZERO_THRESHOLD);
			const double t_robot_stop = (odom_v_norm + 1e-9) / stop_deacc_;
			RCLCPP_WARN(this->get_logger(), "Collision detected: col_step=%d, t_col=%.3lf, t_robot_stop=%.3lf, col_dist_margin=%.3lf", col_step, t_col, t_robot_stop, col_dist_margin);

			if (t_col < 0.0)
			{
				RCLCPP_ERROR(this->get_logger(), "STOPPING (dm=%.3lf, dr=%.3lf, t_col=%.3lf) Current speed: x=%.3lf, y=%.3lf", col_dist_margin, col_step_lin_margin - robot_radius_, t_col, odom_.twist.twist.linear.x, odom_.twist.twist.linear.y);
				emergency_stop = true;
			}
			else if (t_robot_stop + deacc_margin_ /* <-- this could also be a response time for base to start slowing down */ > t_col)
			{
				RCLCPP_WARN(this->get_logger(), "SLOWING DOWN (t_r=%.3lf/t_c=%.3lf)", t_robot_stop, t_col);
				out_cmd.linear.x = 0.0;
				out_cmd.linear.y = 0.0;
				is_slowing = true;
			}
		}
		if (display_path_)
		{
			pub_dwa_path_->publish(dwa_path_);
			dwa_path_.poses.clear();
		}
	}

	if (emergency_stop)
	{
		out_cmd = zero_velocity_;
	}
	else
	{
		out_cmd = limitOutputVelocity(out_cmd, is_slowing);
	}

	auto status = std::make_unique<safety_limiter::msg::Status>();
	status->header.stamp = check_start_time;

	if (is_slowing)
		status->status = safety_limiter::msg::Status::DECELERATING;
	else if (emergency_stop)
		status->status = safety_limiter::msg::Status::STOPPED;
	else
	{
		// col_obstacles.points.clear();
		status->status = safety_limiter::msg::Status::NORMAL;
	}

	pub_obstacles_->publish(col_obstacles);
	pub_status_->publish(std::move(status));

	// If the cmd vel is too old, reset it to zero
	//dirty hack from yuyi
	//t_last_vel_update = ros::Time::now();
	if((this->now().seconds() - t_last_vel_update.seconds() ) > 0.5)
	{
		RCLCPP_WARN(this->get_logger(), "The nav_cmd_ was too old, setting it back to zero velocity!");
		nav_cmd_ = zero_velocity_;
		out_cmd = zero_velocity_;
	}

	pub_vel_->publish(out_cmd);
	if (visualize_vel_vector_)
		visualizeVelocityVectors(out_cmd);
}

/* visualize navigation and odometry velocity vectors */
void Limiter::visualizeVelocityVectors(const geometry_msgs::msg::Twist &_out_vel) const
{
	double vel_angle;
	geometry_msgs::msg::PoseStamped vel_pose;
	vel_pose.header.frame_id = base_link_id;
	vel_pose.header.stamp = odom_.header.stamp;

	vel_angle = atan2(nav_cmd_.linear.y, nav_cmd_.linear.x);
	tf2::Quaternion q;
	q.setRPY(0.0, 0.0, vel_angle);
	geometry_msgs::msg::Quaternion q_msg;
	tf2::convert(q, q_msg);
	vel_pose.pose.orientation = q_msg;
	pub_nav_vel_vector_->publish(vel_pose);
	
	vel_angle = atan2(odom_.twist.twist.linear.y, odom_.twist.twist.linear.x);
	q.setRPY(0.0, 0.0, vel_angle);
	tf2::convert(q, q_msg);
	vel_pose.pose.orientation = q_msg;
	pub_real_vel_vector_->publish(vel_pose);
	
	vel_angle = atan2(_out_vel.linear.y, _out_vel.linear.x);
	q.setRPY(0.0, 0.0, vel_angle);
	tf2::convert(q, q_msg);
	vel_pose.pose.orientation = q_msg;
	pub_out_vel_vector_->publish(vel_pose);
}

// Perform acceleration and deacceleration and limit exceeding velocities to given (linear and angular) limits
geometry_msgs::msg::Twist Limiter::limitOutputVelocity(const geometry_msgs::msg::Twist &_cmd_vel_in, const bool _is_slowing) const
{
	geometry_msgs::msg::Twist cmd_vel_out = _cmd_vel_in;
	const double diff_norm = norm2d(_cmd_vel_in.linear.x - odom_.twist.twist.linear.x, _cmd_vel_in.linear.y - odom_.twist.twist.linear.y);

	// if robot is stopping acceleration is given as stop_deacc_ value (which might be different then acceleration value given otherwise)
	const double acc = _is_slowing ? stop_deacc_ : lin_acc_;
	const double step_dv = acc / 10.0; // known issue, check docs

	if (diff_norm >= step_dv)
	{
		const double lim_x = projectVelocity(odom_.twist.twist.linear.x, _cmd_vel_in.linear.x, acc, step_dv);
		const double lim_y = projectVelocity(odom_.twist.twist.linear.y, _cmd_vel_in.linear.y, acc, step_dv);
		// ROS_INFO("Linear speed limited: %.3lf, %.3lf -> %.3lf, %.3lf", );
		cmd_vel_out.linear.x = lim_x;
		cmd_vel_out.linear.y = lim_y;
	}

	const double vel_norm = norm2d(cmd_vel_out.linear);
	// limit exceeding velocities in x and y direction
	if (vel_norm > max_lin_vel_)
	{
		cmd_vel_out.linear.x = max_lin_vel_ * cmd_vel_out.linear.x / vel_norm;
		cmd_vel_out.linear.y = max_lin_vel_ * cmd_vel_out.linear.y / vel_norm;
	}

	const double z_diff = _cmd_vel_in.angular.z - odom_.twist.twist.angular.z;

	// Inside simulator this works, on real robot odometry reads angular movement even if its only moving linearly
	// apply angular acceeleration and limit exceeding values
	if (limit_ang_acc_)
	{
		if (fabs(z_diff) > ang_acc_ / hz_)
		{
			cmd_vel_out.angular.z = projectVelocity(odom_.twist.twist.angular.z, _cmd_vel_in.angular.z, acc, step_dv);
			// ROS_INFO("Rotational speed limited: %.3lf -> %.3lf", _cmd_vel_in.angular.z, cmd_vel_out.angular.z);
		}
	}

	if (cmd_vel_out.angular.z < -max_ang_vel_)
		cmd_vel_out.angular.z = -max_ang_vel_;
	else if (cmd_vel_out.angular.z > max_ang_vel_)
		cmd_vel_out.angular.z = max_ang_vel_;

	// ROS_INFO("x=%lf,y=%lf,theta=%lf", cmd_vel_out.linear.x, cmd_vel_out.linear.y, cmd_vel_out.angular.z);
	return cmd_vel_out;
}

void Limiter::timerCallback()
{
	static rclcpp::Time t_last_cycle = this->now();  // 初始化为当前时间
	const rclcpp::Time t_now = this->now();

	cycle_interval_ = (t_now - t_last_cycle).seconds();
	t_last_cycle = t_now;

	if (disabled_)
	{
		pub_vel_->publish(nav_cmd_);

		if (visualize_vel_vector_)
			visualizeVelocityVectors(nav_cmd_);
	}
	else
	{
		if (watchdog_cloud_ > 0 && watchdog_odom_ > 0)
		{
			if (speed_limiter_mode_)
			{
				const geometry_msgs::msg::Twist out_cmd = limitOutputVelocity(nav_cmd_);
				pub_vel_->publish(out_cmd);
				if (visualize_vel_vector_)
					visualizeVelocityVectors(out_cmd);
			}
			else
			{
				performCollisionCheck();
			}
		}
		else
		{
			if (watchdog_odom_ < 0)
				RCLCPP_ERROR(this->get_logger(), "Safety_limiter odometry timedout");
			if (watchdog_cloud_ < 0)
				RCLCPP_ERROR(this->get_logger(), "Safety_limiter laser cloud timedout");
			pub_vel_->publish(zero_velocity_);
		}
		--watchdog_vel_;
		--watchdog_cloud_;
		--watchdog_odom_;
	}

	if (odom_.header.frame_id == "odom")
	{
		RCLCPP_INFO_ONCE(this->get_logger(), "ODOMETRY data RECEIVED...CONTROL ACTIVATED");
	}
	else
	{
		RCLCPP_ERROR_ONCE(this->get_logger(), "Robovie ODOMETRY data MISSING!");
	}
}

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Limiter>());
	rclcpp::shutdown();
	return 0;
}
