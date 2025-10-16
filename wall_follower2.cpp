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

  // --- Extract common sensor directions ---
  const double F  = scan_data_[FRONT];
  const double FL = scan_data_[FRONT_LEFT];
  const double LF = scan_data_[LEFT_FRONT];
  const double L  = scan_data_[LEFT];
  const double LB = scan_data_[LEFT_BACK];
  const double FR = scan_data_[FRONT_RIGHT];

  // --- Tunable parameters (更保守一些，提早介入) ---
  const double FRONT_STOP = 0.45;   // 原 0.40 → 提早刹停
  const double FRONT_GO   = 0.70;   // 原 0.60 → 认为“前方开阔”的门槛更高
  const double MIN_BRAKE  = 0.28;   // 防撞瞬时刹停阈值
  const double V_FWD      = 0.20;
  const double V_SLOW     = 0.10;
  const double W_SAFE     = 1.2;
  const double EPS_SIM    = 0.04;
  const double LEFT_SET   = 0.40;
  const double BAND       = 0.03;

  // Handle NaN / Inf readings
  auto finite = [](double x){ return std::isfinite(x); };
  const double Fv  = finite(F)?  F  : 10.0;
  const double FLv = finite(FL)? FL : 10.0;
  const double LFv = finite(LF)? LF : 10.0;
  const double Lv  = finite(L)?  L  : 10.0; (void)Lv;
  const double LBv = finite(LB)? LB : 10.0;
  const double FRv = finite(FR)? FR : 10.0;

  // --- Turn lock / simple state machine ---
  // mode: 0=NONE, 1=BACKING, 2=UTURN
  static int mode = 0;
  static clock::time_point mode_until = clock::time_point::min();
  static double uturn_dir = -1.0; // 掉头默认右转（贴左墙更安全）

  const bool in_mode = (clock::now() < mode_until);

  auto set_mode = [&](int m, double seconds){
    mode = m;
    mode_until = clock::now() + std::chrono::duration_cast<clock::duration>(
      std::chrono::duration<double>(seconds)
    );
  };

  // Stop if the robot is near the start position
  if (near_start) { go(0.0, 0.0); exit(0); }

  // --- 超近距离防撞：硬刹停并立即进入倒退阶段 ---
  if (!in_mode && (Fv < MIN_BRAKE || FLv < MIN_BRAKE || FRv < MIN_BRAKE)) {
    // 立即进入 BACKING 0.30s，然后 UTURN 1.10s
    uturn_dir = -1.0;
    set_mode(1, 0.30);
  }

  // --- 死胡同/直角拐：更果断进入两阶段机动 ---
  // 判据：前方与左右前都近，或两侧前向都近（直角/窄角），提前掉头
  const bool tight_front = (Fv < FRONT_STOP) || (FLv < 0.50 && FRv < 0.50);
  if (!in_mode && tight_front) {
    uturn_dir = -1.0;        // 右转掉头
    set_mode(1, 0.35);       // 先 BACKING 0.35s
  }

  // --- 状态机执行：BACKING -> UTURN ---
  if (in_mode) {
    if (mode == 1) { // BACKING
      go(-0.10, 0.0);
      // BACKING 结束后进入 UTURN（原地自转，不会停住）
      if (clock::now() >= mode_until) {
        // 如果左侧很开阔（凹槽或左拐入口），可改为左转掉头
        if (LFv > LEFT_SET + 0.25 && (LFv - LBv) > 0.18) uturn_dir = +1.0;
        set_mode(2, 1.20); // 自转 1.2s，按底盘角速可调到 1.0~1.4
      }
      return;
    }
    if (mode == 2) { // UTURN
      go(0.0, uturn_dir >= 0.0 ? +W_SAFE : -W_SAFE);

      // 早停条件：前方已明显开阔，且左侧重新稳定（避免转过头）
      const bool front_ok = (Fv > FRONT_GO);
      const bool left_ok  = (std::fabs(LFv - LBv) < 0.06) && (LFv > LEFT_SET - 0.05);
      if (front_ok && left_ok) {
        mode = 0; mode_until = clock::time_point::min();
        return;
      }

      // 正常到时退出 UTURN
      if (clock::now() >= mode_until) {
        mode = 0; mode_until = clock::time_point::min();
      }
      return;
    }
  }

  // --- 预转向：F 介于 STOP 与 GO 时就开始提前修正，避免顶到墙再转 ---
  if (Fv < FRONT_GO && Fv >= FRONT_STOP) {
    if (FLv < FRv - 0.10) {
      go(V_SLOW, -W_SAFE);   // 左近右远 → 提前右修正
      return;
    }
    if (FRv < FLv - 0.10) {
      go(V_SLOW, +W_SAFE);   // 右近左远 → 提前左修正
      return;
    }
  }

  // --- 左侧凹槽（可选）：如果你需要“进凹槽转一圈再出”，可在此加专门机动 ---
  // 此处省略阻塞式机动，交给 BACKING/UTURN 组合来完成更安全的转身

  // --- 正常贴墙：前方开阔 ---
  if (Fv >= FRONT_GO) {
    // 与墙基本平行 → 直行
    if (std::fabs(LFv - LBv) < EPS_SIM) {
      go(V_FWD, 0.0);
      return;
    }
    // 离墙过远 → 左修正
    if (LFv > LEFT_SET + BAND) {
      go(0.12, +0.35);
      return;
    }
    // 离墙过近 → 右修正
    if (LFv < LEFT_SET - BAND) {
      go(0.12, -0.35);
      return;
    }
    // 默认
    go(0.16, 0.0);
    return;
  }

  // --- 前方不足以直行，但还没到极限：交给前面的“预转向/两阶段机动”；若仍到这，保守处理 ---
  if (Fv < FRONT_STOP) {
    // 保险：进入 BACKING+UTURN，而不是边走边擦墙
    uturn_dir = -1.0;
    set_mode(1, 0.30);
    return;
  }

  // --- 拐角：左右前都近 → 果断原地右转（不带 v） ---
  if (FLv < 0.55 && FRv < 0.55) {
    go(0.0, -W_SAFE);
    return;
  }

  // --- 细调：用左后修正 ---
  if (LBv > LEFT_SET + BAND) {
    go(0.10, +0.30);
    return;
  } else if (LBv < LEFT_SET - BAND) {
    go(0.10, -0.30);
    return;
  }

  // --- 防停滞守护：如果什么都不做且离前墙不远，给点右转避免“停住” ---
  if (Fv < 0.90) {
    go(0.05, -0.60);
    return;
  }

  // 默认
  go(0.0, 0.0);
}

void WallFollower::go(double v, double w) {
  const double VMAX = 0.25;
  const double WMAX = 1.8;

  // 裁剪角速度
  if (w >  WMAX) w =  WMAX;
  if (w < -WMAX) w = -WMAX;

  // 曲率自适应降速：转得越急，线速度越低，避免急转时擦墙
  const double curvature = std::min(1.0, std::abs(w) / WMAX);
  const double v_allowed = VMAX * (1.0 - 0.65 * curvature); // 至少留 0.35*VMAX
  if (v >  v_allowed) v =  v_allowed;
  if (v < -0.15) v = -0.15;

  update_cmd_vel(v, w);
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
