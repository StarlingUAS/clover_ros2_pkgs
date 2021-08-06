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

// #include <GeographicLib/Geodesic.hpp> needed for global to local transformations but cannot find it...

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
        void publish();

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

        // Wait Triggers
        bool wait_for_offboard_trigger = false;
        bool wait_for_arm_trigger = false;
        bool wait_for_land_trigger = false;

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
};

SimpleOffboard::SimpleOffboard() :
	Node("simple_offboard",
		 "",
		 rclcpp::NodeOptions()
			.allow_undeclared_parameters(true)
			.automatically_declare_parameters_from_overrides(true)
	)
{
    // Initialise transforms
    this->tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf2_ros::TransformListener tf_listener(*this->tf_buffer);
    transform_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
	static_transform_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

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
    this->get_parameter_or("body_frame", this->body.child_frame_id, string("body"));
	this->get_parameters("reference_frames", this->reference_frames);

    // Get Timeout parameters
    setpoint_rate = this->get_timeout_parameter("setpoint_rate", 30.0, true);
    state_timeout = this->get_timeout_parameter("state_timeout", 3.0);
	local_position_timeout = this->get_timeout_parameter("local_position_timeout", 2.0);
	velocity_timeout = this->get_timeout_parameter("velocity_timeout", 2.0);
	global_position_timeout = this->get_timeout_parameter("global_position_timeout", 10.0);
	battery_timeout = this->get_timeout_parameter("battery_timeout", 2.0);
	manual_control_timeout = this->get_timeout_parameter("manual_control_timeout", 0.0);
	transform_timeout = this->get_timeout_parameter("transform_timeout", 0.5);
	telemetry_transform_timeout = this->get_timeout_parameter("telemetry_transform_timeout", 0.5);
	offboard_timeout = this->get_timeout_parameter("offboard_timeout", 3.0);
	land_timeout = this->get_timeout_parameter("land_timeout", 3.0);
	arming_timeout = this->get_timeout_parameter("arming_timeout", 4.0);

    // Initialise Service Clients
    this->arming = this->create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");
    this->set_mode = this->create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");

    // Initialise Telemetry Subscribers
    this->state_sub =           this->create_subscription<mavros_msgs::msg::State>(
        "mavros/state", 1, [this](const mavros_msgs::msg::State::SharedPtr s){this->handleState(s);});
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
        "mavros/local_position/pose", 1, [this](const PoseStamped::SharedPtr s){this->handleLocalPosition(s);});

    // Initialise Publishers
    position_pub     = this->create_publisher<PoseStamped>("mavros/setpoint_position/local", 1);
    position_raw_pub = this->create_publisher<PositionTarget>("mavros/setpoint_raw/local", 1);
    attitude_pub     = this->create_publisher<PoseStamped>("mavros/setpoint_attitude/attitude", 1);
    attitude_raw_pub = this->create_publisher<AttitudeTarget>("mavros/setpoint_raw/attitude", 1);
    rates_pub        = this->create_publisher<TwistStamped>("mavros/setpoint_attitude/cmd_vel", 1);
    thrust_pub       = this->create_publisher<Thrust>("mavros/setpoint_attitude/thrust", 1);

    // Initialise Service Servers
    gt_serv = this->create_service<clover_ros2::srv::GetTelemetry>("get_telemetry", std::bind(&SimpleOffboard::getTelemetry, this, _1, _2));
    na_serv = this->create_service<clover_ros2::srv::Navigate>("navigate", std::bind(&SimpleOffboard::navigate, this, _1, _2));
    ng_serv = this->create_service<clover_ros2::srv::NavigateGlobal>("navigate_global", std::bind(&SimpleOffboard::navigateGlobal, this, _1, _2));
    sp_serv = this->create_service<clover_ros2::srv::SetPosition>("set_position", std::bind(&SimpleOffboard::setPosition, this, _1, _2));
    sv_serv = this->create_service<clover_ros2::srv::SetVelocity>("set_velocity", std::bind(&SimpleOffboard::setVelocity, this, _1, _2));
    sa_serv = this->create_service<clover_ros2::srv::SetAttitude>("set_attitude", std::bind(&SimpleOffboard::setAttitude, this, _1, _2));
    sr_serv = this->create_service<clover_ros2::srv::SetRates>("set_rates", std::bind(&SimpleOffboard::setRates, this, _1, _2));
    ld_serv = this->create_service<std_srvs::srv::Trigger>("land", std::bind(&SimpleOffboard::land, this, _1, _2));

    // Initialise Timers
    this->setpoint_timer = this->create_wall_timer(this->setpoint_rate, [this](){this->publish();});

    // Initialise Internal State
    this->position_msg.header.frame_id = this->local_frame;
	this->position_raw_msg.header.frame_id = this->local_frame;
	this->position_raw_msg.coordinate_frame = PositionTarget::FRAME_LOCAL_NED;
	this->rates_msg.header.frame_id = this->fcu_frame;

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

void SimpleOffboard::wait_for_offboard(){
    this->wait_for_offboard_trigger = false;
    auto start = this->now();
    rclcpp::TimerBase::SharedPtr timer = this->create_wall_timer(
        std::chrono::duration<double>(0.1),
        [this, start, timer](){
            if(this->state->mode == "OFFBOARD") {
                this->wait_for_offboard_trigger = true;
                timer->cancel();
            } else if (this->now() - start > this->offboard_timeout) {
                string report = "OFFBOARD timed out";
                if ((start - this->statustext->header.stamp).seconds() < 0.0)
					report += ": " + this->statustext->text;
                timer->cancel();
                this->wait_for_offboard_trigger = true;
				throw std::runtime_error(report);
            }
        }
    );
    while(!this->wait_for_offboard_trigger) {}
}

void SimpleOffboard::wait_for_arm(){
    this->wait_for_arm_trigger = false;
    auto start = this->now();
    rclcpp::TimerBase::SharedPtr timer = this->create_wall_timer(
        std::chrono::duration<double>(0.1),
        [this, start, timer](){
            if(this->state->armed) {
                this->wait_for_arm_trigger = true;
                timer->cancel();
            } else if (this->now() - start > this->arming_timeout) {
                string report = "Arming timed out";
                if ((start - this->statustext->header.stamp).seconds() < 0.0)
					report += ": " + this->statustext->text;
                this->wait_for_arm_trigger = true;
                timer->cancel();
				throw std::runtime_error(report);
            }
        }
    );
    while(!this->wait_for_arm_trigger) {}
}

void SimpleOffboard::wait_for_land(){
    this->wait_for_land_trigger = false;
    auto start = this->now();
    rclcpp::TimerBase::SharedPtr timer = this->create_wall_timer(
        std::chrono::duration<double>(0.01),
        [this, start, timer](){
            if(this->state->mode == "AUTO.LAND") {
                this->wait_for_land_trigger = true;
                timer->cancel();
            } else if (this->now() - start > this->land_timeout) {
                this->wait_for_land_trigger = true;
                timer->cancel();
				throw std::runtime_error("Land request timed out");
            }
        }
    );
    while(!this->wait_for_land_trigger) {}

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

void SimpleOffboard::publish() {
    auto stamp = this->now();

    if (this->setpoint_type == NONE) return;

    this->position_raw_msg.header.stamp = stamp;
	this->thrust_msg.header.stamp = stamp;
	this->rates_msg.header.stamp = stamp;

    try {
        auto _transform_delay = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(0.05));
		// transform position and/or yaw
		if (this->setpoint_type == NAVIGATE || this->setpoint_type == NAVIGATE_GLOBAL || this->setpoint_type == POSITION || this->setpoint_type == VELOCITY || this->setpoint_type == ATTITUDE) {
			this->setpoint_position.header.stamp = stamp;
			this->tf_buffer->transform(this->setpoint_position, this->setpoint_position_transformed, this->local_frame, _transform_delay);
		}

		// transform velocity
		if (this->setpoint_type == VELOCITY) {
			this->setpoint_velocity.header.stamp = stamp;
			this->tf_buffer->transform(this->setpoint_velocity, this->setpoint_velocity_transformed, this->local_frame, _transform_delay);
		}

	} catch (const tf2::TransformException& e) {
		RCLCPP_WARN(this->get_logger(), "publish can't transform");
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
    return true;
}

bool SimpleOffboard::getTelemetry(std::shared_ptr<GetTelemetry::Request> req, std::shared_ptr<GetTelemetry::Response> res)
{
    return true;
}

bool SimpleOffboard::navigate(std::shared_ptr<Navigate::Request> req, std::shared_ptr<Navigate::Response> res) {
	return this->serve(NAVIGATE, req->x, req->y, req->z, NAN, NAN, NAN, NAN, NAN, req->yaw, NAN, NAN, req->yaw_rate, NAN, NAN, NAN, req->speed, req->frame_id, req->auto_arm, res->success, res->message);
}

bool SimpleOffboard::navigateGlobal(std::shared_ptr<NavigateGlobal::Request> req, std::shared_ptr<NavigateGlobal::Response> res) {
    return this->serve(NAVIGATE_GLOBAL, NAN, NAN, req->z, NAN, NAN, NAN, NAN, NAN, req->yaw, NAN, NAN, req->yaw_rate, req->lat, req->lon, NAN, req->speed, req->frame_id, req->auto_arm, res->success, res->message);
}

bool SimpleOffboard::setPosition(std::shared_ptr<SetPosition::Request> req, std::shared_ptr<SetPosition::Response> res) {
	return this->serve(POSITION, req->x, req->y, req->z, NAN, NAN, NAN, NAN, NAN, req->yaw, NAN, NAN, req->yaw_rate, NAN, NAN, NAN, NAN, req->frame_id, req->auto_arm, res->success, res->message);
}

bool SimpleOffboard::setVelocity(std::shared_ptr<SetVelocity::Request> req, std::shared_ptr<SetVelocity::Response> res) {
	return this->serve(VELOCITY, NAN, NAN, NAN, req->vx, req->vy, req->vz, NAN, NAN, req->yaw, NAN, NAN, req->yaw_rate, NAN, NAN, NAN, NAN, req->frame_id, req->auto_arm, res->success, res->message);
}

bool SimpleOffboard::setAttitude(std::shared_ptr<SetAttitude::Request> req, std::shared_ptr<SetAttitude::Response> res) {
	return this->serve(ATTITUDE, NAN, NAN, NAN, NAN, NAN, NAN, req->pitch, req->roll, req->yaw, NAN, NAN, NAN, NAN, NAN, req->thrust, NAN, req->frame_id, req->auto_arm, res->success, res->message);
}

bool SimpleOffboard::setRates(std::shared_ptr<SetRates::Request> req, std::shared_ptr<SetRates::Response> res) {
	return this->serve(RATES, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, req->pitch_rate, req->roll_rate, req->yaw_rate, NAN, NAN, req->thrust, NAN, "", req->auto_arm, res->success, res->message);
}

bool SimpleOffboard::land(std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
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

        auto result = this->set_mode->async_send_request(sm);
        // Wait for the result.
        // if (rclcpp::spin_until_future_complete(this, result) != rclcpp::FutureReturnCode::SUCCESS){
        //     RCLCPP_ERROR(this->get_logger(), "Failed to call service set mode");
        // }

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
    rclcpp::spin(std::make_shared<SimpleOffboard>());
    rclcpp::shutdown();
    return 0;
}