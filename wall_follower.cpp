// Released under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
// Author: Claude Sammut
// Last Modified: 2024.10.14

// Use this code as the basis for a wall follower

#include "wall_follower/wall_follower.hpp"

#include <memory>


namespace {
  constexpr double kMaxLinear     = 0.15;  
  constexpr double kMaxAngular    = 1.8;
  constexpr double kFrontStopDist = 0.22; 
  constexpr double kFollowDistRef = 0.60;  
  constexpr int    kBeamWidthDeg  = 15;   

  inline bool valid_range(double r, double rmin, double rmax) {
    return std::isfinite(r) && r > rmin + 1e-3 && r < rmax - 1e-3;
  }
}


using namespace std::chrono_literals;

WallFollower::WallFollower()
: Node("wall_follower_node")
{
	/************************************************************
	** Initialise variables
	************************************************************/
	for (int i = 0; i < 12; i++)
		scan_data_[i] = 0.0;

	robot_pose_ = 0.0;
	near_start = false;

	/************************************************************
	** Initialise ROS publishers and subscribers
	************************************************************/
	auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

	// Initialise publishers
	cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", qos);

	// Initialise subscribers
	scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
		"scan", \
		rclcpp::SensorDataQoS(), \
		std::bind(
			&WallFollower::scan_callback, \
			this, \
			std::placeholders::_1));
	odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
		"odom", qos, std::bind(&WallFollower::odom_callback, this, std::placeholders::_1));

	/************************************************************
	** Initialise ROS timers
	************************************************************/
	update_timer_ = this->create_wall_timer(10ms, std::bind(&WallFollower::update_callback, this));

	RCLCPP_INFO(this->get_logger(), "Wall follower node has been initialised");
}

WallFollower::~WallFollower()
{
	RCLCPP_INFO(this->get_logger(), "Wall follower node has been terminated");
}

/********************************************************************************
** Callback functions for ROS subscribers
********************************************************************************/

#define START_RANGE	0.2

void WallFollower::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
	static bool first = true;
	static bool start_moving = true;

	tf2::Quaternion q(
		msg->pose.pose.orientation.x,
		msg->pose.pose.orientation.y,
		msg->pose.pose.orientation.z,
		msg->pose.pose.orientation.w);
	tf2::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);

	robot_pose_ = yaw;

	double current_x =  msg->pose.pose.position.x;
	double current_y =  msg->pose.pose.position.y;
	if (first)
	{
		start_x = current_x;
		start_y = current_y;
		first = false;
	}
	else if (start_moving)
	{
		if (fabs(current_x - start_x) > START_RANGE || fabs(current_y - start_y) > START_RANGE)
			start_moving = false;
	}
	else if (fabs(current_x - start_x) < START_RANGE && fabs(current_y - start_y) < START_RANGE)
	{
		fprintf(stderr, "Near start!!\n");
		near_start = true;
		first = true;
		start_moving = true;
	}
}

#define BEAM_WIDTH 15


void WallFollower::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{

  const int scan_angle_deg[12] = {0, 30, 60, 90, 120, 150, 180, 210, 240, 270, 300, 330};

  auto angle_to_index = [&](double deg) -> int {
    const double rad = deg * M_PI / 180.0;
    int idx = static_cast<int>(std::round((rad - msg->angle_min) / msg->angle_increment));
    if (idx < 0) idx = 0;
    if (idx >= static_cast<int>(msg->ranges.size())) idx = static_cast<int>(msg->ranges.size()) - 1;
    return idx;
  };

  auto sector_min = [&](double center_deg, int half_width_deg) -> double {
    const int c = angle_to_index(center_deg);
    const int w = std::max(1, static_cast<int>(std::round(half_width_deg / (msg->angle_increment * 180.0/M_PI))));
    const int lo = std::max(0, c - w);
    const int hi = std::min(static_cast<int>(msg->ranges.size()) - 1, c + w);
    double best = msg->range_max;
    for (int i = lo; i <= hi; ++i) {
      const double r = msg->ranges[i];
      if (valid_range(r, msg->range_min, msg->range_max) && r < best) best = r;
    }
    return best;
  };


  const double front_left_half  = sector_min(+kBeamWidthDeg * 0.5, kBeamWidthDeg);
  const double front_right_half = sector_min(-kBeamWidthDeg * 0.5, kBeamWidthDeg);
  scan_data_[0] = std::min(front_left_half, front_right_half);

  for (int i = 1; i < 12; ++i) {
    scan_data_[i] = sector_min(scan_angle_deg[i], kBeamWidthDeg);
  }
}


void WallFollower::update_cmd_vel(double linear, double angular)
{
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x  = std::clamp(linear,  -kMaxLinear,  kMaxLinear);
  cmd_vel.angular.z = std::clamp(angular, -kMaxAngular, kMaxAngular);
  cmd_vel_pub_->publish(cmd_vel);
}

/********************************************************************************
** Update functions
********************************************************************************/

bool pl_near;


void WallFollower::update_callback()
{
  if (near_start) { update_cmd_vel(0.0, 0.0); exit(0); }

  const double front = scan_data_[FRONT]; 
  if (front < kFrontStopDist) {
    update_cmd_vel(0.0, 0.0);
    return;
  }

  const double lf = scan_data_[LEFT_FRONT];
  const double fl = scan_data_[FRONT_LEFT];
  const double fr = scan_data_[FRONT_RIGHT];


  if (lf > 1.0) {
  
    update_cmd_vel(0.12, +1.2);
  }
  else if (front < 0.50) {
   
    update_cmd_vel(0.0, -1.2);
  }
  else if (fl < kFollowDistRef) {
  
    update_cmd_vel(0.12, -1.0);
  }
  else if (fr < kFollowDistRef) {
    ）
    update_cmd_vel(0.12, +1.0);
  }
  else if (lf > 0.75) {
  
    update_cmd_vel(0.12, +0.8);
  }
  else {
    update_cmd_vel(0.15, 0.0);
  }
}





/*******************************************************************************
** Main
*******************************************************************************/
int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<WallFollower>());
	rclcpp::shutdown();

	return 0;
}
