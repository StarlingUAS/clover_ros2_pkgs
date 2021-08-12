/*
 * Simplified copter control in OFFBOARD mode
 * Copyright (C) 2019 Copter Express Technologies
 *
 * Original Author: Oleg Kalachev <okalachev@gmail.com>
 * Ported to ROS2 by Mickey Li <mickey.li@bristol.ac.uk> (University of Bristol)
 *
 * Distributed under MIT License (available at https://opensource.org/licenses/MIT).
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 */
#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <memory>
#include <boost/algorithm/string.hpp>
#include <algorithm>
#include <stdexcept>

#include <GeographicLib/Geodesic.hpp> // needed for global to local transformations but cannot find it...

#include <tf2/utils.h>
#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <std_srvs/srv/trigger.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>

#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/battery_state.hpp>

#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/msg/position_target.hpp"
#include "mavros_msgs/msg/attitude_target.hpp"
#include "mavros_msgs/msg/thrust.hpp"
#include "mavros_msgs/msg/status_text.hpp"
#include "mavros_msgs/msg/manual_control.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/set_mode.hpp"

#include "clover_ros2/srv/set_led_effect.hpp"
#include "clover_ros2/srv/get_telemetry.hpp"
#include "clover_ros2/srv/navigate.hpp"
#include "clover_ros2/srv/navigate_global.hpp"
#include "clover_ros2/srv/set_position.hpp"
#include "clover_ros2/srv/set_velocity.hpp"
#include "clover_ros2/srv/set_attitude.hpp"
#include "clover_ros2/srv/set_rates.hpp"

using std::string;
using std::isnan;
using namespace geometry_msgs::msg;
using namespace sensor_msgs::msg;
using namespace clover_ros2::srv;
using std::placeholders::_1;
using std::placeholders::_2;
using mavros_msgs::msg::PositionTarget;
using mavros_msgs::msg::AttitudeTarget;
using mavros_msgs::msg::Thrust;


#define ENSURE_FINITE(var) { if (!std::isfinite(var)) throw std::runtime_error(#var " argument cannot be NaN or Inf"); }
#define TIMEOUT(time_now, msg, timeout) (time_now - msg.header.stamp > timeout)

enum setpoint_type_t {
	NONE,
	NAVIGATE,
	NAVIGATE_GLOBAL,
	POSITION,
	VELOCITY,
	ATTITUDE,
	RATES
};


enum setpoint_yaw_type_t { YAW, YAW_RATE, TOWARDS };

inline double hypot(double x, double y, double z)
{
	return std::sqrt(x * x + y * y + z * z);
}

inline float getDistance(const Point& from, const Point& to)
{
	return hypot(from.x - to.x, from.y - to.y, from.z - to.z);
}

class SimpleOffboard : public rclcpp::Node
{
    public:
        SimpleOffboard();
        bool getTelemetry(std::shared_ptr<GetTelemetry::Request> req, std::shared_ptr<GetTelemetry::Response> res);
        bool navigate(std::shared_ptr<Navigate::Request> req, std::shared_ptr<Navigate::Response> res);
        bool navigateGlobal(std::shared_ptr<NavigateGlobal::Request> req, std::shared_ptr<NavigateGlobal::Response> res);
        bool setPosition(std::shared_ptr<SetPosition::Request> req, std::shared_ptr<SetPosition::Response> res);
        bool setVelocity(std::shared_ptr<SetVelocity::Request> req, std::shared_ptr<SetVelocity::Response> res);
        bool setAttitude(std::shared_ptr<SetAttitude::Request> req, std::shared_ptr<SetAttitude::Response> res);
        bool setRates(std::shared_ptr<SetRates::Request> req, std::shared_ptr<SetRates::Response> res);
        bool land(std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res);

    private:

        void handleState(const mavros_msgs::msg::State::SharedPtr s);
        void handleLocalPosition(const PoseStamped::SharedPtr pose);
        void publishBodyFrame();
        void publish(const rclcpp::Time& stamp);
        PoseStamped globalToLocal(double lat, double lon);
        void sendSetModeRequest(string custom_mode);

        void getNavigateSetpoint(const rclcpp::Time& stamp, float speed, Point& nav_setpoint);
        void checkManualControl();
        void checkState();

        bool serve(enum setpoint_type_t sp_type, float x, float y, float z, float vx, float vy, float vz,
                    float pitch, float roll, float yaw, float pitch_rate, float roll_rate, float yaw_rate, // editorconfig-checker-disable-line
                    float lat, float lon, float thrust, float speed, string frame_id, bool auto_arm, // editorconfig-checker-disable-line
                    bool& success, string& message); // editorconfig-checker-disable-line

        // Helper utility functions
        std::chrono::duration<double> get_timeout_parameter(string name, double default_param, bool invert = false);
        void wait_for_offboard();
        void wait_for_arm();
        void wait_for_land();
        bool checkTransformExistsBlocking(const string& target_frame, const string& source_frame, const rclcpp::Time& time, const rclcpp::Duration& timeout);
        void publishSetpoint(const rclcpp::Time& stamp, bool auto_arm);
        void restartSetpointTimer();

        // Last received telemetry messages
        std::shared_ptr<mavros_msgs::msg::State> state;
        std::shared_ptr<mavros_msgs::msg::StatusText> statustext;
        std::shared_ptr<mavros_msgs::msg::ManualControl> manual_control;
        std::shared_ptr<PoseStamped> local_position;
        std::shared_ptr<TwistStamped> velocity;
        std::shared_ptr<NavSatFix> global_position;
        std::shared_ptr<BatteryState> battery;

        // tf2
        std::shared_ptr<tf2_ros::Buffer> tf_buffer;
        std::shared_ptr<tf2_ros::TransformListener> transform_listener;
        std::shared_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster;
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_transform_broadcaster;

        // Parameters
        string local_frame;
        string fcu_frame;
        std::chrono::duration<double> setpoint_rate;
        std::chrono::duration<double> transform_timeout; // May want to convert to std::chrono::duration?
        std::chrono::duration<double> telemetry_transform_timeout;
        std::chrono::duration<double> offboard_timeout;
        std::chrono::duration<double> land_timeout;
        std::chrono::duration<double> arming_timeout;
        std::chrono::duration<double> local_position_timeout;
        std::chrono::duration<double> state_timeout;
        std::chrono::duration<double> velocity_timeout;
        std::chrono::duration<double> global_position_timeout;
        std::chrono::duration<double> battery_timeout;
        std::chrono::duration<double> manual_control_timeout;
        float default_speed;
        bool auto_release;
        bool land_only_in_offboard, nav_from_sp, check_kill_switch;
        std::map<string, string> reference_frames;

        // Containers
        enum setpoint_type_t setpoint_type = NONE;
        enum setpoint_yaw_type_t setpoint_yaw_type;
        rclcpp::TimerBase::SharedPtr setpoint_timer;
        tf2::Quaternion tq;
        PoseStamped position_msg;
        PositionTarget position_raw_msg;
        AttitudeTarget att_raw_msg;
        Thrust thrust_msg;
        TwistStamped rates_msg;
        TransformStamped target, setpoint;
        geometry_msgs::msg::TransformStamped body;

        // State
        PoseStamped nav_start;
        PoseStamped setpoint_position, setpoint_position_transformed;
        Vector3Stamped setpoint_velocity, setpoint_velocity_transformed;
        QuaternionStamped setpoint_attitude, setpoint_attitude_transformed;
        float setpoint_yaw_rate;
        float nav_speed;
        bool busy = false;
        bool wait_armed = false;
        bool nav_from_sp_flag = false;

        // Service Clients
        rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr        arming;
        rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr            set_mode;

        // Publishers
        rclcpp::Publisher<PoseStamped>::SharedPtr                       position_pub;
        rclcpp::Publisher<PositionTarget>::SharedPtr                    position_raw_pub;
        rclcpp::Publisher<PoseStamped>::SharedPtr                       attitude_pub;
        rclcpp::Publisher<AttitudeTarget>::SharedPtr                    attitude_raw_pub;
        rclcpp::Publisher<TwistStamped>::SharedPtr                      rates_pub;
        rclcpp::Publisher<Thrust>::SharedPtr                            thrust_pub;

        // Telemetry Subscribers
        rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr         state_sub;
        rclcpp::Subscription<TwistStamped>::SharedPtr                    velocity_sub;
        rclcpp::Subscription<NavSatFix>::SharedPtr                       global_position_sub;
        rclcpp::Subscription<BatteryState>::SharedPtr                    battery_sub;
        rclcpp::Subscription<mavros_msgs::msg::StatusText>::SharedPtr    statustext_sub;
        rclcpp::Subscription<mavros_msgs::msg::ManualControl>::SharedPtr manual_control_sub;
        rclcpp::Subscription<PoseStamped>::SharedPtr                     local_position_sub;

        // Service Servers
        rclcpp::Service<clover_ros2::srv::GetTelemetry>::SharedPtr      gt_serv;
        rclcpp::Service<clover_ros2::srv::Navigate>::SharedPtr          na_serv;
        rclcpp::Service<clover_ros2::srv::NavigateGlobal>::SharedPtr    ng_serv;
        rclcpp::Service<clover_ros2::srv::SetPosition>::SharedPtr       sp_serv;
        rclcpp::Service<clover_ros2::srv::SetVelocity>::SharedPtr       sv_serv;
        rclcpp::Service<clover_ros2::srv::SetAttitude>::SharedPtr       sa_serv;
        rclcpp::Service<clover_ros2::srv::SetRates>::SharedPtr          sr_serv;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr              ld_serv;

        rclcpp::CallbackGroup::SharedPtr callback_group_services_;
        rclcpp::CallbackGroup::SharedPtr callback_group_timers_;
        rclcpp::CallbackGroup::SharedPtr callback_group_clients_;
        rclcpp::CallbackGroup::SharedPtr callback_group_subscribers_;

};

SimpleOffboard::SimpleOffboard() :
	Node("simple_offboard",
		 "",
		 rclcpp::NodeOptions()
			.allow_undeclared_parameters(true)
			.automatically_declare_parameters_from_overrides(true)
	)
{

    this->callback_group_services_ = this->create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);
    this->callback_group_timers_ = this->create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);
    this->callback_group_subscribers_ = this->create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);
    this->callback_group_clients_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

    // Initialise transforms
    this->tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    this->transform_listener = std::make_shared<tf2_ros::TransformListener>(*this->tf_buffer);
    this->transform_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
	this->static_transform_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    // Get parameters
    this->get_parameter_or("mavros/local_position/tf/frame_id", this->local_frame, string("map"));
	this->get_parameter_or("mavros/local_position/tf/child_frame_id", this->fcu_frame, string("base_link"));
    this->get_parameter_or("target_frame", this->target.child_frame_id, string("navigate_target"));
    this->get_parameter_or("setpoint", this->setpoint.child_frame_id, string("setpoint"));
	this->get_parameter_or("auto_release", this->auto_release, true);
	this->get_parameter_or("land_only_in_offboard", this->land_only_in_offboard, true);
	this->get_parameter_or("nav_from_sp", this->nav_from_sp, true);
	this->get_parameter_or("check_kill_switch", this->check_kill_switch, true);
	this->get_parameter_or("default_speed", this->default_speed, 0.5f);
    this->get_parameter_or("body_frame", this->body.child_frame_id, string("base_link"));
	this->get_parameters("reference_frames", this->reference_frames);

    // Get Timeout parameters
    setpoint_rate = this->get_timeout_parameter("setpoint_rate", 30.0, true);
    state_timeout = this->get_timeout_parameter("state_timeout", 3.0);
	local_position_timeout = this->get_timeout_parameter("local_position_timeout", 2.0);
	velocity_timeout = this->get_timeout_parameter("velocity_timeout", 2.0);
	global_position_timeout = this->get_timeout_parameter("global_position_timeout", 10.0);
	battery_timeout = this->get_timeout_parameter("battery_timeout", 2.0);
	manual_control_timeout = this->get_timeout_parameter("manual_control_timeout", 0.0);
	transform_timeout = this->get_timeout_parameter("transform_timeout", 2.0);
	telemetry_transform_timeout = this->get_timeout_parameter("telemetry_transform_timeout", 0.5);
	offboard_timeout = this->get_timeout_parameter("offboard_timeout", 3.0);
	land_timeout = this->get_timeout_parameter("land_timeout", 3.0);
	arming_timeout = this->get_timeout_parameter("arming_timeout", 4.0);

    // Initialise Service Clients
    this->arming = this->create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming", rmw_qos_profile_services_default, this->callback_group_clients_);
    this->set_mode = this->create_client<mavros_msgs::srv::SetMode>("mavros/set_mode", rmw_qos_profile_services_default, this->callback_group_clients_);

    // Initialise Telemetry Subscribers
    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = this->callback_group_subscribers_;

    this->state_sub =           this->create_subscription<mavros_msgs::msg::State>(
        "mavros/state", 1, [this](const mavros_msgs::msg::State::SharedPtr s){this->handleState(s);}, sub_opt);
    this->velocity_sub =        this->create_subscription<TwistStamped>(
        "mavros/local_position/velocity_body", 1, [this](const TwistStamped::SharedPtr msg){this->velocity = msg;});
    this->global_position_sub = this->create_subscription<NavSatFix>(
        "mavros/global_position/global", 1, [this](const NavSatFix::SharedPtr msg){this->global_position = msg;});
    this->battery_sub =         this->create_subscription<BatteryState>(
        "mavros/battery", 1, [this](const BatteryState::SharedPtr msg){this->battery = msg;});
    this->statustext_sub =      this->create_subscription<mavros_msgs::msg::StatusText>(
        "mavros/statustext/recv", 1, [this](const mavros_msgs::msg::StatusText::SharedPtr msg){this->statustext = msg;});
    this->manual_control_sub =  this->create_subscription<mavros_msgs::msg::ManualControl>(
        "mavros/manual_control/control", 1, [this](const mavros_msgs::msg::ManualControl::SharedPtr msg){this->manual_control = msg;});
    this->local_position_sub =  this->create_subscription<PoseStamped>(
        "mavros/local_position/pose", 1, [this](const PoseStamped::SharedPtr s){this->handleLocalPosition(s);}, sub_opt);

    // Initialise Publishers
    position_pub     = this->create_publisher<PoseStamped>("mavros/setpoint_position/local", 1);
    position_raw_pub = this->create_publisher<PositionTarget>("mavros/setpoint_raw/local", 1);
    attitude_pub     = this->create_publisher<PoseStamped>("mavros/setpoint_attitude/attitude", 1);
    attitude_raw_pub = this->create_publisher<AttitudeTarget>("mavros/setpoint_raw/attitude", 1);
    rates_pub        = this->create_publisher<TwistStamped>("mavros/setpoint_attitude/cmd_vel", 1);
    thrust_pub       = this->create_publisher<Thrust>("mavros/setpoint_attitude/thrust", 1);

    // Initialise Service Servers
    gt_serv = this->create_service<clover_ros2::srv::GetTelemetry>("get_telemetry", std::bind(&SimpleOffboard::getTelemetry, this, _1, _2));
    na_serv = this->create_service<clover_ros2::srv::Navigate>("navigate", std::bind(&SimpleOffboard::navigate, this, _1, _2), rmw_qos_profile_services_default, this->callback_group_services_);
    ng_serv = this->create_service<clover_ros2::srv::NavigateGlobal>("navigate_global", std::bind(&SimpleOffboard::navigateGlobal, this, _1, _2));
    sp_serv = this->create_service<clover_ros2::srv::SetPosition>("set_position", std::bind(&SimpleOffboard::setPosition, this, _1, _2));
    sv_serv = this->create_service<clover_ros2::srv::SetVelocity>("set_velocity", std::bind(&SimpleOffboard::setVelocity, this, _1, _2));
    sa_serv = this->create_service<clover_ros2::srv::SetAttitude>("set_attitude", std::bind(&SimpleOffboard::setAttitude, this, _1, _2));
    sr_serv = this->create_service<clover_ros2::srv::SetRates>("set_rates", std::bind(&SimpleOffboard::setRates, this, _1, _2));
    ld_serv = this->create_service<std_srvs::srv::Trigger>("land", std::bind(&SimpleOffboard::land, this, _1, _2));

    // Initialise Internal State
    this->position_msg.header.frame_id = this->local_frame;
	this->position_raw_msg.header.frame_id = this->local_frame;
	this->position_raw_msg.coordinate_frame = PositionTarget::FRAME_LOCAL_NED;
	this->rates_msg.header.frame_id = this->fcu_frame;

    this->publish(this->now());

    RCLCPP_INFO(this->get_logger(), "Simple Offboard Controller Initialised");
}

inline std::chrono::duration<double> SimpleOffboard::get_timeout_parameter(string name, double default_param, bool invert) {
    double t;
    this->get_parameter_or(name, t, default_param);
    if(invert) {
        return std::chrono::duration<double>(1.0/t);
    }
    return std::chrono::duration<double>(t);
}

void SimpleOffboard::sendSetModeRequest(string custom_mode) {
    auto sm = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    sm->custom_mode = custom_mode;

    while (!this->set_mode->wait_for_service(std::chrono::duration<int>(2))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for set mode service. Exiting.");
            throw std::runtime_error("Interrupted while waiting for set mode service. Exiting.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
    }

    auto result_future = this->set_mode->async_send_request(sm);
    if (result_future.wait_for(std::chrono::duration<double>(10.0)) != std::future_status::ready)
    {
        RCLCPP_ERROR(this->get_logger(), "service call failed :(");
        throw std::runtime_error("Service call failed");
        return;
    }

    auto result = result_future.get();

    RCLCPP_INFO(this->get_logger(), "Sent request: %s", custom_mode.c_str());
}

void SimpleOffboard::wait_for_offboard(){
    if (this->state->mode == "OFFBOARD") {
        RCLCPP_INFO(this->get_logger(), "Already Offboard Mode");
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Sending OFFBOARD Request");
    this->sendSetModeRequest("OFFBOARD");

    auto start = this->now();
    while(this->state->mode != "OFFBOARD") {
        RCLCPP_INFO(this->get_logger(), "Waiting for Offboard");
        if (this->now() - start > this->offboard_timeout) {
            RCLCPP_ERROR(this->get_logger(), "Offboard timed out");
            throw std::runtime_error("Offboard timed out");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    RCLCPP_INFO(this->get_logger(), "Confirmed In OFFBOARD Mode");
}

void SimpleOffboard::wait_for_arm(){
    if (this->state->armed) {
        RCLCPP_INFO(this->get_logger(), "Already Armed");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Sending ARM Request");
    auto sm = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    sm->value = true;
    while (!this->arming->wait_for_service(std::chrono::duration<int>(2))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for set mode service. Exiting.");
            throw std::runtime_error("Interrupted while waiting for set mode service. Exiting.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
    }

    auto result_future = this->arming->async_send_request(sm);
    if (result_future.wait_for(std::chrono::duration<double>(10.0)) != std::future_status::ready)
    {
        RCLCPP_ERROR(this->get_logger(), "Arming service call failed :(");
        throw std::runtime_error("Arming Service call failed");
        return;
    }

    auto result = result_future.get();
    RCLCPP_INFO(this->get_logger(), "Sent ARM Request");

    auto start = this->now();
    while(!this->state->armed) {
        RCLCPP_INFO(this->get_logger(), "Waiting for Arming");
        if (this->now() - start > this->arming_timeout) {
            RCLCPP_ERROR(this->get_logger(), "Arming timed out");
            throw std::runtime_error("Arming timed out");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    RCLCPP_INFO(this->get_logger(), "Confirmed Armed");
}

void SimpleOffboard::wait_for_land(){
    auto start = this->now();
    while(this->state->mode != "AUTO.LAND") {
        RCLCPP_INFO(this->get_logger(), "Waiting for Landing");
        if (this->now() - start > this->land_timeout) {
            RCLCPP_ERROR(this->get_logger(), "Landing timed out");
            throw std::runtime_error("Landing timed out");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    RCLCPP_INFO(this->get_logger(), "Confirmed Land Mode");
}

bool SimpleOffboard::checkTransformExistsBlocking(const string& target_frame, const string& source_frame, const rclcpp::Time& time, const rclcpp::Duration& timeout) {
    // RCLCPP_INFO(this->get_logger(), "Testing transform from %s to %s", target_frame.c_str(), source_frame.c_str());
    std::string error;

    while (this->now() - time < timeout) {
        if(this->tf_buffer->canTransform(target_frame, source_frame, time)) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return false;
}

void SimpleOffboard::restartSetpointTimer() {
    if (this->setpoint_timer) {
        this->setpoint_timer->cancel();
    }
    this->setpoint_timer = this->create_wall_timer(this->setpoint_rate, [this](){this->publish(this->now());});
    RCLCPP_DEBUG(this->get_logger(), "Reset Setpoint timer");
}

PoseStamped SimpleOffboard::globalToLocal(double lat, double lon)
{
	auto earth = GeographicLib::Geodesic::WGS84();

	// Determine azimuth and distance between current and destination point
	double _, distance, azimuth;
	earth.Inverse(this->global_position->latitude, this->global_position->longitude, lat, lon, distance, _, azimuth);

	double x_offset, y_offset;
	double azimuth_radians = azimuth * M_PI / 180;
	x_offset = distance * sin(azimuth_radians);
	y_offset = distance * cos(azimuth_radians);

    this->checkTransformExistsBlocking(this->local_frame, this->fcu_frame, this->global_position->header.stamp, std::chrono::duration<double>(0.2));

	auto local = this->tf_buffer->lookupTransform(this->local_frame, this->fcu_frame, this->global_position->header.stamp);

	PoseStamped pose;
	pose.header.stamp = this->global_position->header.stamp; // TODO: ?
	pose.header.frame_id = this->local_frame;
	pose.pose.position.x = local.transform.translation.x + x_offset;
	pose.pose.position.y = local.transform.translation.y + y_offset;
	pose.pose.orientation.w = 1;
	return pose;
}

void SimpleOffboard::handleState(const mavros_msgs::msg::State::SharedPtr s)
{
	this->state = s;
	if (s->mode != "OFFBOARD") {
		// flight intercepted
		this->nav_from_sp_flag = false;
	}
}

void SimpleOffboard::handleLocalPosition(const PoseStamped::SharedPtr pose)
{
	this->local_position = pose;
	this->publishBodyFrame();
	// TODO: terrain?, home?
}

inline void SimpleOffboard::publishBodyFrame()
{
	if (this->body.child_frame_id.empty()) return;

	tf2::Quaternion q;
	q.setRPY(0, 0, tf2::getYaw(this->local_position->pose.orientation));
    this->body.transform.rotation = tf2::toMsg(q);

	this->body.transform.translation.x = this->local_position->pose.position.x;
	this->body.transform.translation.y = this->local_position->pose.position.y;
	this->body.transform.translation.z = this->local_position->pose.position.z;
	this->body.header.frame_id = this->local_position->header.frame_id;
	this->body.header.stamp = this->local_position->header.stamp;
	this->transform_broadcaster->sendTransform(this->body);
}

void SimpleOffboard::publish(const rclcpp::Time& stamp) {
    if (this->setpoint_type == NONE) return;

    this->position_raw_msg.header.stamp = stamp;
	this->thrust_msg.header.stamp = stamp;
	this->rates_msg.header.stamp = stamp;

    try {
        auto _transform_delay = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(0.05));
		// transform position and/or yaw
		if (this->setpoint_type == NAVIGATE || this->setpoint_type == NAVIGATE_GLOBAL || this->setpoint_type == POSITION || this->setpoint_type == VELOCITY || this->setpoint_type == ATTITUDE) {
			this->setpoint_position.header.stamp = stamp;
            // RCLCPP_INFO(this->get_logger(), "Transform from %s to %s", this->setpoint_position.header.frame_id.c_str(), this->local_frame.c_str());
            // RCLCPP_INFO(this->get_logger(), "Setpoint is (%f, %f, %f)", this->setpoint_position.pose.position.x, this->setpoint_position.pose.position.y, this->setpoint_position.pose.position.z);
            this->checkTransformExistsBlocking(this->setpoint_position.header.frame_id, this->local_frame, this->now(), this->transform_timeout);
			this->tf_buffer->transform(this->setpoint_position, this->setpoint_position_transformed, this->local_frame, std::chrono::duration_cast<std::chrono::nanoseconds>(this->transform_timeout));
            // RCLCPP_INFO(this->get_logger(), "Setpoint Transformed is (%f, %f, %f)", this->setpoint_position_transformed.pose.position.x, this->setpoint_position_transformed.pose.position.y, this->setpoint_position_transformed.pose.position.z);
		}

		// transform velocity
		if (this->setpoint_type == VELOCITY) {
			this->setpoint_velocity.header.stamp = stamp;
			this->tf_buffer->transform(this->setpoint_velocity, this->setpoint_velocity_transformed, this->local_frame, _transform_delay);
		}

	} catch (const tf2::TransformException& e) {
		RCLCPP_WARN(this->get_logger(), "publish can't transform, error: %s", e.what());
	}

    if (this->setpoint_type == NAVIGATE || this->setpoint_type == NAVIGATE_GLOBAL) {


		this->position_msg.pose.orientation = this->setpoint_position_transformed.pose.orientation; // copy yaw
		this->getNavigateSetpoint(stamp, this->nav_speed, this->position_msg.pose.position);

		if (this->setpoint_yaw_type == TOWARDS) {
			double yaw_towards = atan2(this->position_msg.pose.position.y - this->nav_start.pose.position.y,
			                           this->position_msg.pose.position.x - this->nav_start.pose.position.x);
            tf2::Quaternion q;
            q.setRPY(0, 0, yaw_towards);
			this->position_msg.pose.orientation = tf2::toMsg(q);
        }
	}

    if (this->setpoint_type == POSITION) {
		this->position_msg = this->setpoint_position_transformed;
	}

    if (this->setpoint_type == POSITION || this->setpoint_type == NAVIGATE || this->setpoint_type == NAVIGATE_GLOBAL) {
		this->position_msg.header.stamp = stamp;

		if (setpoint_yaw_type == YAW || setpoint_yaw_type == TOWARDS) {
			this->position_pub->publish(this->position_msg);

		} else {
			this->position_raw_msg.type_mask = PositionTarget::IGNORE_VX +
                                                PositionTarget::IGNORE_VY +
                                                PositionTarget::IGNORE_VZ +
                                                PositionTarget::IGNORE_AFX +
                                                PositionTarget::IGNORE_AFY +
                                                PositionTarget::IGNORE_AFZ +
                                                PositionTarget::IGNORE_YAW;
			this->position_raw_msg.yaw_rate = this->setpoint_yaw_rate;
			this->position_raw_msg.position = this->position_msg.pose.position;
			this->position_raw_pub->publish(this->position_raw_msg);
		}

		// publish setpoint frame
		if (!this->setpoint.child_frame_id.empty()) {
			this->setpoint.transform.translation.x = this->position_msg.pose.position.x;
			this->setpoint.transform.translation.y = this->position_msg.pose.position.y;
			this->setpoint.transform.translation.z = this->position_msg.pose.position.z;
			this->setpoint.transform.rotation = this->position_msg.pose.orientation;
			this->setpoint.header.frame_id = this->position_msg.header.frame_id;
			this->setpoint.header.stamp = this->position_msg.header.stamp;
			this->transform_broadcaster->sendTransform(this->setpoint);
		}

        // RCLCPP_INFO(this->get_logger(), "Currently at (%f, %f, %f), going to (%f, %f, %f)", this->local_position->pose.position.x, this->local_position->pose.position.y, this->local_position->pose.position.z, this->position_msg.pose.position.x, this->position_msg.pose.position.y, this->position_msg.pose.position.z);
	}

    if (this->setpoint_type == ATTITUDE) {
		this->attitude_pub->publish(this->setpoint_position_transformed);
		this->thrust_pub->publish(this->thrust_msg);
	}

	if (this->setpoint_type == RATES) {
		// this->rates_pub.publish(this->rates_msg);
		// this->thrust_pub.publish(this->thrust_msg);
		// mavros rates topics waits for rates in local frame
		// use rates in body frame for simplicity
		this->att_raw_msg.header.stamp = stamp;
		this->att_raw_msg.header.frame_id = this->fcu_frame;
		this->att_raw_msg.type_mask = AttitudeTarget::IGNORE_ATTITUDE;
		this->att_raw_msg.body_rate = this->rates_msg.twist.angular;
		this->att_raw_msg.thrust = this->thrust_msg.thrust;
		this->attitude_raw_pub->publish(this->att_raw_msg);
	}
}

void SimpleOffboard::getNavigateSetpoint(const rclcpp::Time& stamp, float speed, Point& nav_setpoint)
{
	if (this->wait_armed) {
		// don't start navigating if we're waiting arming
		this->nav_start.header.stamp = stamp;
	}

	float distance = getDistance(this->nav_start.pose.position, this->setpoint_position_transformed.pose.position);
	float time = distance / speed;
	float passed = std::min((stamp - this->nav_start.header.stamp).seconds() / time, 1.0);

	nav_setpoint.x = this->nav_start.pose.position.x + (this->setpoint_position_transformed.pose.position.x - this->nav_start.pose.position.x) * passed;
	nav_setpoint.y = this->nav_start.pose.position.y + (this->setpoint_position_transformed.pose.position.y - this->nav_start.pose.position.y) * passed;
	nav_setpoint.z = this->nav_start.pose.position.z + (this->setpoint_position_transformed.pose.position.z - this->nav_start.pose.position.z) * passed;
}

inline void SimpleOffboard::checkManualControl()
{
	if (this->manual_control_timeout != std::chrono::duration<double>::zero() && (this->now() - this->manual_control->header.stamp) > this->manual_control_timeout) {
		throw std::runtime_error("Manual control timeout, RC is switched off?");
	}

	if (this->check_kill_switch) {
		// switch values: https://github.com/PX4/PX4-Autopilot/blob/c302514a0809b1765fafd13c014d705446ae1113/msg/manual_control_setpoint.msg#L3
		const uint8_t SWITCH_POS_NONE = 0; // switch is not mapped
		const uint8_t SWITCH_POS_ON = 1; // switch activated
		const uint8_t SWITCH_POS_MIDDLE = 2; // middle position
		const uint8_t SWITCH_POS_OFF = 3; // switch not activated

		const int KILL_SWITCH_BIT = 12; // https://github.com/PX4/Firmware/blob/c302514a0809b1765fafd13c014d705446ae1113/src/modules/mavlink/mavlink_messages.cpp#L3975
		uint8_t kill_switch = (this->manual_control->buttons & (0b11 << KILL_SWITCH_BIT)) >> KILL_SWITCH_BIT;

		if (kill_switch == SWITCH_POS_ON)
			throw std::runtime_error("Kill switch is on");
	}
}

inline void SimpleOffboard::checkState()
{
	if ( (this->now() - this->state->header.stamp) > this->state_timeout )
		throw std::runtime_error("State timeout, check mavros settings");

	if (!this->state->connected)
		throw std::runtime_error("No connection to FCU, https://clover.coex.tech/connection");
}

bool SimpleOffboard::serve(enum setpoint_type_t sp_type, float x, float y, float z, float vx, float vy, float vz,
           float pitch, float roll, float yaw, float pitch_rate, float roll_rate, float yaw_rate, // editorconfig-checker-disable-line
           float lat, float lon, float thrust, float speed, string frame_id, bool auto_arm, // editorconfig-checker-disable-line
           bool& success, string& message) // editorconfig-checker-disable-line
{
    auto stamp = this->now();

    try {
        if (this->busy)
			throw std::runtime_error("Busy");

		this->busy = true;

        // Checks
        this->checkState();

        if (auto_arm && this->manual_control) {
            this->checkManualControl();
        }

        // default frame is local frame
        if (frame_id.empty())
            frame_id = this->local_frame;

        // look up for reference frame
        auto search = this->reference_frames.find(frame_id);
        const string& reference_frame = search == reference_frames.end() ? frame_id: search->second;

        bool skip = false;

        // Serve "partial" commands
        if (!auto_arm && std::isfinite(yaw) &&
		    isnan(x) && isnan(y) && isnan(z) && isnan(vx) && isnan(vy) && isnan(vz) &&
		    isnan(pitch) && isnan(roll) && isnan(thrust) &&
		    isnan(lat) && isnan(lon)) {

            RCLCPP_INFO(this->get_logger(), "Serving Partial 1");

            if (this->setpoint_type == POSITION || this->setpoint_type == NAVIGATE || this->setpoint_type == NAVIGATE_GLOBAL || this->setpoint_type == VELOCITY) {
                this->checkTransformExistsBlocking(this->setpoint_position.header.frame_id, frame_id, stamp, this->transform_timeout);

				message = "Changing yaw only";

				QuaternionStamped q;
				q.header.frame_id = frame_id;
				q.header.stamp = stamp;
                tf2::Quaternion quat; // TODO: pitch=0, roll=0 is not totally correct
                quat.setRPY(0.0, 0.0, yaw);
				q.quaternion = tf2::toMsg(quat);
				this->setpoint_position.pose.orientation = this->tf_buffer->transform(q, this->setpoint_position.header.frame_id).quaternion;
				this->setpoint_yaw_type = YAW;
                this->publishSetpoint(stamp, auto_arm);
                skip = true;
			} else {
				throw std::runtime_error("Setting yaw is possible only when position or velocity setpoints active");
			}
        }

        if (!auto_arm && std::isfinite(yaw_rate) &&
		    isnan(x) && isnan(y) && isnan(z) && isnan(vx) && isnan(vy) && isnan(vz) &&
		    isnan(pitch) && isnan(roll) && isnan(yaw) && isnan(thrust) &&
		    isnan(lat) && isnan(lon)) {

            RCLCPP_INFO(this->get_logger(), "Serving Partial 2");
			// change only the yaw rate
			if (this->setpoint_type == POSITION || this->setpoint_type == NAVIGATE || this->setpoint_type == NAVIGATE_GLOBAL || this->setpoint_type == VELOCITY) {
				message = "Changing yaw rate only";

				this->setpoint_yaw_type = YAW_RATE;
				this->setpoint_yaw_rate = yaw_rate;
                this->publishSetpoint(stamp, auto_arm);
                skip = true;
			} else {
				throw std::runtime_error("Setting yaw rate is possible only when position or velocity setpoints active");
			}
        }

        RCLCPP_INFO(this->get_logger(), "Serving Normal 1");

        if (!skip) { // Refactor out goto statements.
            // Serve normal commands
            if (sp_type == NAVIGATE || sp_type == POSITION) {
                ENSURE_FINITE(x);
                ENSURE_FINITE(y);
                ENSURE_FINITE(z);
            } else if (sp_type == NAVIGATE_GLOBAL) {
                ENSURE_FINITE(lat);
                ENSURE_FINITE(lon);
                ENSURE_FINITE(z);
            } else if (sp_type == VELOCITY) {
                ENSURE_FINITE(vx);
                ENSURE_FINITE(vy);
                ENSURE_FINITE(vz);
            } else if (sp_type == ATTITUDE) {
                ENSURE_FINITE(pitch);
                ENSURE_FINITE(roll);
                ENSURE_FINITE(thrust);
            } else if (sp_type == RATES) {
                ENSURE_FINITE(pitch_rate);
                ENSURE_FINITE(roll_rate);
                ENSURE_FINITE(thrust);
            }

            if (sp_type == NAVIGATE || sp_type == NAVIGATE_GLOBAL) {
                if (this->now() - this->local_position->header.stamp > this->local_position_timeout)
                    throw std::runtime_error("No local position, check settings");

                if (speed < 0)
                    throw std::runtime_error("Navigate speed must be positive, " + std::to_string(speed) + " passed");

                if (speed == 0)
                    speed = default_speed;
            }

            if (sp_type == NAVIGATE || sp_type == NAVIGATE_GLOBAL || sp_type == POSITION || sp_type == VELOCITY) {
                if (yaw_rate != 0 && !std::isnan(yaw))
                    throw std::runtime_error("Yaw value should be NaN for setting yaw rate");

                if (std::isnan(yaw_rate) && std::isnan(yaw))
                    throw std::runtime_error("Both yaw and yaw_rate cannot be NaN");
            }

            if (sp_type == NAVIGATE_GLOBAL) {
                if (this->now() - this->global_position->header.stamp > this->global_position_timeout)
                    throw std::runtime_error("No global position");
            }

            if (sp_type == NAVIGATE || sp_type == NAVIGATE_GLOBAL || sp_type == POSITION || sp_type == VELOCITY || sp_type == ATTITUDE) {
                // make sure transform from frame_id to reference frame available
                this->checkTransformExistsBlocking(reference_frame, frame_id, stamp, this->transform_timeout);

                // make sure transform from reference frame to local frame available
                this->checkTransformExistsBlocking(local_frame, reference_frame, stamp, this->transform_timeout);
            }

            if (sp_type == NAVIGATE_GLOBAL) {
            	// Calculate x and from lat and lot in request's frame
            	auto pose_local = this->globalToLocal(lat, lon);
            	pose_local.header.stamp = stamp; // TODO: fix
            	auto xy_in_req_frame = this->tf_buffer->transform(pose_local, frame_id);
            	x = xy_in_req_frame.pose.position.x;
            	y = xy_in_req_frame.pose.position.y;
            }

            // Everything fine - switch setpoint type
            this->setpoint_type = sp_type;

            if (sp_type != NAVIGATE && sp_type != NAVIGATE_GLOBAL) {
                this->nav_from_sp_flag = false;
            }

            if (sp_type == NAVIGATE || sp_type == NAVIGATE_GLOBAL) {
                // starting point
                if (this->nav_from_sp && this->nav_from_sp_flag) {
                    message = "Navigating from current setpoint";
                    this->nav_start = this->position_msg;
                } else {
                    this->nav_start = *this->local_position;
                }
                this->nav_speed = speed;
                this->nav_from_sp_flag = true;
            }

            if (sp_type == POSITION || sp_type == NAVIGATE || sp_type == NAVIGATE_GLOBAL || sp_type == VELOCITY || sp_type == ATTITUDE) {
                // destination point and/or yaw
                PoseStamped ps;
                ps.header.frame_id = frame_id;
                ps.header.stamp = stamp;
                ps.pose.position.x = x;
                ps.pose.position.y = y;
                ps.pose.position.z = z;
                ps.pose.orientation.w = 1.0; // Ensure quaternion is always valid

                if (std::isnan(yaw)) {
                    this->setpoint_yaw_type = YAW_RATE;
                    this->setpoint_yaw_rate = yaw_rate;
                } else if (std::isinf(yaw) && yaw > 0) {
                    // yaw towards
                    this->setpoint_yaw_type = TOWARDS;
                    yaw = 0;
                    this->setpoint_yaw_rate = 0;
                } else {
                    this->setpoint_yaw_type = YAW;
                    this->setpoint_yaw_rate = 0;
                    tf2::Quaternion q;
                    q.setRPY(0, 0, yaw);
                    ps.pose.orientation = tf2::toMsg(q);
                }

                this->tf_buffer->transform(ps, this->setpoint_position, reference_frame);
            }

            if (sp_type == VELOCITY) {
                Vector3Stamped vel;
                vel.header.frame_id = frame_id;
                vel.header.stamp = stamp;
                vel.vector.x = vx;
                vel.vector.y = vy;
                vel.vector.z = vz;
                this->tf_buffer->transform(vel, this->setpoint_velocity, reference_frame);
            }

            if (sp_type == ATTITUDE || sp_type == RATES) {
                this->thrust_msg.thrust = thrust;
            }

            if (sp_type == RATES) {
                this->rates_msg.twist.angular.x = roll_rate;
                this->rates_msg.twist.angular.y = pitch_rate;
                this->rates_msg.twist.angular.z = yaw_rate;
            }

            this->wait_armed = auto_arm;
            this->publishSetpoint(stamp, auto_arm);
        }
    } catch (const std::exception& e) {
		message = e.what();
		RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
		this->busy = false;
		return false;
	}

	success = true;
	this->busy = false;
	return true;
}

void SimpleOffboard::publishSetpoint(const rclcpp::Time& stamp, bool auto_arm) {
    RCLCPP_INFO(this->get_logger(), "Publishing Setpoint");
    this->publish(stamp); // calculate initial transformed messages first
    this->restartSetpointTimer();

    // publish target frame
    if (!this->target.child_frame_id.empty()) {
        if (this->setpoint_type == NAVIGATE || this->setpoint_type == NAVIGATE_GLOBAL || this->setpoint_type == POSITION) {
            this->target.header.frame_id = this->setpoint_position.header.frame_id;
            this->target.header.stamp = stamp;
            this->target.transform.translation.x = this->setpoint_position.pose.position.x;
            this->target.transform.translation.y = this->setpoint_position.pose.position.y;
            this->target.transform.translation.z = this->setpoint_position.pose.position.z;
            this->target.transform.rotation = this->setpoint_position.pose.orientation;
            this->static_transform_broadcaster->sendTransform(this->target);
        }
    }

    if (auto_arm) {
        RCLCPP_INFO(this->get_logger(), "Auto Arm, waiting for offboard and arm");
        this->wait_for_offboard(); // Check offboard
        this->wait_for_arm();      // Arm drone
        this->wait_armed = false;
    } else if (this->state->mode != "OFFBOARD") {
        this->setpoint_timer->cancel();
        throw std::runtime_error("Copter is not in OFFBOARD mode, use auto_arm?");
    } else if (!this->state->armed) {
        this->setpoint_timer->cancel();
        throw std::runtime_error("Copter is not armed, use auto_arm?");
    }
}

bool SimpleOffboard::getTelemetry(std::shared_ptr<GetTelemetry::Request> req, std::shared_ptr<GetTelemetry::Response> res)
{
    auto stamp = this->now();

    if (req->frame_id.empty())
		req->frame_id = this->local_frame;

	res->frame_id = req->frame_id;
	res->x = NAN;
	res->y = NAN;
	res->z = NAN;
	res->lat = NAN;
	res->lon = NAN;
	res->alt = NAN;
	res->vx = NAN;
	res->vy = NAN;
	res->vz = NAN;
	res->pitch = NAN;
	res->roll = NAN;
	res->yaw = NAN;
	res->pitch_rate = NAN;
	res->roll_rate = NAN;
	res->yaw_rate = NAN;
	res->voltage = NAN;
	res->cell_voltage = NAN;

    if(stamp - this->state->header.stamp < this->state_timeout) {
        res->connected = this->state->connected;
        res->armed = this->state->armed;
        res->mode = this->state->mode;
    }

    try {
		this->checkTransformExistsBlocking(req->frame_id, this->fcu_frame, stamp, this->telemetry_transform_timeout);
		auto transform = this->tf_buffer->lookupTransform(req->frame_id, this->fcu_frame, stamp);
		res->x = transform.transform.translation.x;
		res->y = transform.transform.translation.y;
		res->z = transform.transform.translation.z;

		double yaw, pitch, roll;
		tf2::getEulerYPR(transform.transform.rotation, yaw, pitch, roll);
		res->yaw = yaw;
		res->pitch = pitch;
		res->roll = roll;
	} catch (const tf2::TransformException& e) {
		RCLCPP_DEBUG(this->get_logger(), "Get Telemetry Error %s", e.what());
	}

    if (stamp - this->velocity->header.stamp < this->velocity_timeout) {
		try {
			// transform velocity
			this->checkTransformExistsBlocking(req->frame_id, this->fcu_frame, this->velocity->header.stamp, this->telemetry_transform_timeout);
			Vector3Stamped vec, vec_out;
			vec.header.stamp = this->velocity->header.stamp;
			vec.header.frame_id = this->velocity->header.frame_id;
			vec.vector = this->velocity->twist.linear;
			this->tf_buffer->transform(vec, vec_out, req->frame_id);

			res->vx = vec_out.vector.x;
			res->vy = vec_out.vector.y;
			res->vz = vec_out.vector.z;
		} catch (const tf2::TransformException& e) {}

		// use angular velocities as they are
		res->yaw_rate =      this->velocity->twist.angular.z;
		res->pitch_rate =    this->velocity->twist.angular.y;
		res->roll_rate =     this->velocity->twist.angular.x;
	}

    if (stamp - this->global_position->header.stamp < this->global_position_timeout) {
		res->lat = this->global_position->latitude;
		res->lon = this->global_position->longitude;
		res->alt = this->global_position->altitude;
	}

	if (stamp - this->battery->header.stamp < this->battery_timeout) {
		res->voltage = this->battery->voltage;
		if (!this->battery->cell_voltage.empty()) {
            double voltage = 0;
            for (float v: this->battery->cell_voltage) {
                voltage += v;
            }
			res->cell_voltage = voltage; // total voltage
		}
	}

    return true;
}

bool SimpleOffboard::navigate(std::shared_ptr<Navigate::Request> req, std::shared_ptr<Navigate::Response> res) {
    RCLCPP_INFO(this->get_logger(), "Received Navigate Request to (%f, %f, %f, yaw: %f) speed: %f, frame: %s, auto_arm: %s", req->x, req->y, req->z, req->yaw, req->speed, req->frame_id.c_str(), req->auto_arm?"true":"false");
	return this->serve(NAVIGATE, req->x, req->y, req->z, NAN, NAN, NAN, NAN, NAN, req->yaw, NAN, NAN, req->yaw_rate, NAN, NAN, NAN, req->speed, req->frame_id, req->auto_arm, res->success, res->message);
}

bool SimpleOffboard::navigateGlobal(std::shared_ptr<NavigateGlobal::Request> req, std::shared_ptr<NavigateGlobal::Response> res) {
    RCLCPP_INFO(this->get_logger(), "Received Navigate Global Request to (lat: %f, lon: %f, z: %f, yaw: %f) speed: %f, frame: %s, auto_arm: %s", req->lat, req->lon, req->z, req->yaw, req->speed, req->frame_id.c_str(), req->auto_arm?"true":"false");
    return this->serve(NAVIGATE_GLOBAL, NAN, NAN, req->z, NAN, NAN, NAN, NAN, NAN, req->yaw, NAN, NAN, req->yaw_rate, req->lat, req->lon, NAN, req->speed, req->frame_id, req->auto_arm, res->success, res->message);
}

bool SimpleOffboard::setPosition(std::shared_ptr<SetPosition::Request> req, std::shared_ptr<SetPosition::Response> res) {
    RCLCPP_INFO(this->get_logger(), "Received Set Position Request to (%f, %f, %f, yaw: %f), frame: %s, auto_arm: %s", req->x, req->y, req->z, req->yaw, req->frame_id.c_str(), req->auto_arm?"true":"false");
	return this->serve(POSITION, req->x, req->y, req->z, NAN, NAN, NAN, NAN, NAN, req->yaw, NAN, NAN, req->yaw_rate, NAN, NAN, NAN, NAN, req->frame_id, req->auto_arm, res->success, res->message);
}

bool SimpleOffboard::setVelocity(std::shared_ptr<SetVelocity::Request> req, std::shared_ptr<SetVelocity::Response> res) {
    RCLCPP_INFO(this->get_logger(), "Received Set Velocity Request to (%f, %f, %f, yaw: %f), frame: %s, auto_arm: %s", req->vx, req->vy, req->vz, req->yaw, req->frame_id.c_str(), req->auto_arm?"true":"false");
	return this->serve(VELOCITY, NAN, NAN, NAN, req->vx, req->vy, req->vz, NAN, NAN, req->yaw, NAN, NAN, req->yaw_rate, NAN, NAN, NAN, NAN, req->frame_id, req->auto_arm, res->success, res->message);
}

bool SimpleOffboard::setAttitude(std::shared_ptr<SetAttitude::Request> req, std::shared_ptr<SetAttitude::Response> res) {
    RCLCPP_INFO(this->get_logger(), "Received Set Attitude Request to (p: %f, r: %f, y: %f, thrust: %f), frame: %s, auto_arm: %s", req->pitch, req->roll, req->yaw, req->thrust, req->frame_id.c_str(), req->auto_arm?"true":"false");
	return this->serve(ATTITUDE, NAN, NAN, NAN, NAN, NAN, NAN, req->pitch, req->roll, req->yaw, NAN, NAN, NAN, NAN, NAN, req->thrust, NAN, req->frame_id, req->auto_arm, res->success, res->message);
}

bool SimpleOffboard::setRates(std::shared_ptr<SetRates::Request> req, std::shared_ptr<SetRates::Response> res) {
    RCLCPP_INFO(this->get_logger(), "Received Set Rates Request to (p: %f, r: %f, y: %f, thrust: %f), auto_arm: %s", req->pitch_rate, req->roll_rate, req->yaw_rate, req->thrust, req->auto_arm?"true":"false");
	return this->serve(RATES, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, req->pitch_rate, req->roll_rate, req->yaw_rate, NAN, NAN, req->thrust, NAN, "", req->auto_arm, res->success, res->message);
}

bool SimpleOffboard::land(std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    RCLCPP_INFO(this->get_logger(), "Received Land Request");
    try {
		if (this->busy)
			throw std::runtime_error("Busy");

		this->busy = true;

		this->checkState();

		if (this->land_only_in_offboard) {
			if (this->state->mode != "OFFBOARD") {
				throw std::runtime_error("Copter is not in OFFBOARD mode");
			}
		}

		auto sm = std::make_shared<mavros_msgs::srv::SetMode::Request>();
		sm->custom_mode = "AUTO.LAND";

        while (!this->set_mode->wait_for_service(std::chrono::duration<int>(2))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for set mode service. Exiting.");
                return 0;
            }
            RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
        }

        auto result_future = this->set_mode->async_send_request(sm);
        if (result_future.wait_for(std::chrono::duration<double>(10.0)) != std::future_status::ready)
        {
            RCLCPP_ERROR(this->get_logger(), "Landing service call failed :(");
            throw std::runtime_error("Landing Service call failed");
            return 0;
        }

        auto result = result_future.get();

		this->wait_for_land();
        busy = false;
        return true;

	} catch (const std::exception& e) {
		res->message = e.what();
		RCLCPP_INFO(this->get_logger(), "%s", e.what());
		busy = false;
		return true;
	}
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto offboard = std::make_shared<SimpleOffboard>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(offboard);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}