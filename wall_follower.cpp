// Released under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
// Author: Claude Sammut
// Last Modified: 2024.10.14

// Use this code as the basis for a wall follower

#include "wall_follower/wall_follower.hpp"

#include <memory>


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
	uint16_t scan_angle[12] = {0, 30, 60, 90, 120, 150, 180, 210, 240, 270, 300, 330};

	double closest = msg->range_max;
	for (int angle = 360-BEAM_WIDTH; angle < 360; angle++)
		if (msg->ranges.at(angle) < closest)
			closest = msg->ranges.at(angle);
	for (int angle = 0; angle < BEAM_WIDTH; angle++)
		if (msg->ranges.at(angle) < closest)
			closest = msg->ranges.at(angle);
	scan_data_[0] = closest;

	for (int i = 1; i < 12; i++)
	{
		closest = msg->range_max;
		for (int angle = scan_angle[i]-BEAM_WIDTH; angle < scan_angle[i]+BEAM_WIDTH; angle++)
			if (msg->ranges.at(angle) < closest)
				closest = msg->ranges.at(angle);
		scan_data_[i] = closest;
	}
}

void WallFollower::update_cmd_vel(double linear, double angular)
{
	geometry_msgs::msg::Twist cmd_vel;
	cmd_vel.linear.x = linear;
	cmd_vel.angular.z = angular;

	cmd_vel_pub_->publish(cmd_vel);
}

/********************************************************************************
** Update functions
********************************************************************************/

bool pl_near;


void WallFollower::update_callback()
{
  using clock = std::chrono::steady_clock;

  // --- Turn lock: prevent sudden cancellation of turning
  // when the front suddenly becomes clear ---
  static clock::time_point lock_until = clock::time_point::min();
  static double lock_dir = 0.0; // +1 = turning left, -1 = turning right
  const bool locked = (clock::now() < lock_until);

  // Helper: initiate a locked turn for a duration
  auto lock_turn = [&](double seconds, double v, double w_sign){
	lock_dir = (w_sign >= 0.0) ? +1.0 : -1.0;
	lock_until = clock::now() + std::chrono::duration<double>(seconds);
	// Use go() so limits and W_SAFE apply consistently
	go(v, (w_sign >= 0.0 ? +W_SAFE : -W_SAFE));
  };


  // Helper: safely send velocity command with limits
  auto go = [&](double v, double w){
    const double VMAX = 0.25;
    const double WMAX = 1.8;
    if (v >  VMAX) v =  VMAX;
    if (v < -0.15) v = -0.15;
    if (w >  WMAX) w =  WMAX;
    if (w < -WMAX) w = -WMAX;
    update_cmd_vel(v, w);
  };

  // Stop if the robot is near the start position
  if (near_start) { go(0.0, 0.0); exit(0); }

  // --- Extract common sensor directions ---
  const double F  = scan_data_[FRONT];
  const double FL = scan_data_[FRONT_LEFT];
  const double LF = scan_data_[LEFT_FRONT];
  const double L  = scan_data_[LEFT];
  const double LB = scan_data_[LEFT_BACK];
  const double FR = scan_data_[FRONT_RIGHT];

  // --- Tunable parameters ---
  const double FRONT_STOP = 0.40;   // distance to stop
  const double FRONT_GO   = 0.60;   // distance = “front is open”
  const double V_FWD      = 0.20;   // normal forward speed
  const double V_SLOW     = 0.10;   // slow speed
  const double W_SAFE     = 1.2;    // safe angular velocity
  const double EPS_SIM    = 0.04;   // tolerance for “similar distance”
  const double LEFT_SET   = 0.40;   // ideal wall distance on the left
  const double BAND       = 0.03;   // hysteresis band

  // Handle NaN / Inf readings to prevent logic flickering
  auto finite = [](double x){ return std::isfinite(x); };
  const double Fv  = finite(F)?  F  : 10.0;
  const double FLv = finite(FL)? FL : 10.0;
  const double LFv = finite(LF)? LF : 10.0;
  const double Lv  = finite(L)?  L  : 10.0;
  const double LBv = finite(LB)? LB : 10.0;
  const double FRv = finite(FR)? FR : 10.0;

  // --- Emergency: obstacle directly ahead or front-right ---
  if (!locked && (Fv < FRONT_STOP || FRv < FRONT_STOP)) {
    lock_turn(0.5, 0.0, -W_SAFE);  // rotate right for 0.5s
    return;
  }

  // --- During turn lock: keep turning smoothly ---
  if (locked) {
    go(V_SLOW, 0.8 * (lock_dir >= 0.0 ? +1.0 : -1.0));
    return;
  }

  // --- Clear front area ---
  if (Fv >= FRONT_GO) {
    // Left-front and left-back similar → wall parallel → go straight
    if (std::fabs(LFv - LBv) < EPS_SIM) {
      go(V_FWD, 0.0);
      return;
    }
    // Too far from wall → move left
    if (LFv > LEFT_SET + BAND) {
      go(0.1, +0.3);
      return;
    }
    // Too close to wall → move right
    if (LFv < LEFT_SET - BAND) {
      go(0.1, -0.3);
      return;
    }
    // Default: move forward slowly
    go(0.15, 0.0);
    return;
  }

  // --- Not enough front clearance ---
  if (Fv < FRONT_STOP) {
    // Too close: stop and turn right
    go(0.0, -W_SAFE);
    return;
  }

  // Both front-left and front-right close → corner → turn right
  if (FLv < 0.5 && FRv < 0.5) {
    go(V_SLOW, -W_SAFE);
    return;
  }

  // Adjust based on left-back (fine-tuning)
  if (LBv > LEFT_SET + BAND) {
    go(0.1, +0.3); // wall drifting away → left correction
    return;
  } else if (LBv < LEFT_SET - BAND) {
    go(0.1, -0.3); // wall too close → right correction
    return;
  }

  // Default stop (should rarely happen)
  go(0.0, 0.0);
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
