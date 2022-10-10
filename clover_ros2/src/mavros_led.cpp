#include <chrono>
#include <memory>
#include <boost/algorithm/string.hpp>

#include "rclcpp/rclcpp.hpp"
#include "clover_ros2_msgs/srv/set_led_effect.hpp"

// #include "sensor_msgs/msg/battery_state.h"
#include "mavros_msgs/msg/state.hpp"
#include "std_msgs/msg/empty.hpp"

using namespace std;

class MavrosLEDController: public rclcpp::Node
{
    public:
        MavrosLEDController();

    private:
        void handleMavrosState(const mavros_msgs::msg::State::SharedPtr msg);

        void parse_event_params();
        void check_connection_cb();
        void emergency_stop();

        void apply_event_effect(const string& event);
        bool send_effect(const clover_ros2_msgs::srv::SetLEDEffect::Request::SharedPtr effect, bool wait=false);


        // Parameters
        std::map<string, clover_ros2_msgs::srv::SetLEDEffect::Request::SharedPtr> event_effect_map; // Maps Mavros Event 

        // Set Effect Service Client
        rclcpp::Client<clover_ros2_msgs::srv::SetLEDEffect>::SharedPtr set_effect_client;

        // Mavros State
        rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr mavros_state_sub;
        std::shared_ptr<mavros_msgs::msg::State> mavros_state;

        // Topic Monitors
        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr            emergency_stop_sub;

        bool connected;
        std::chrono::duration<double> connection_check_rate;
        rclcpp::TimerBase::SharedPtr check_timer; // Periodically check mavros has updated
        rclcpp::Time mavros_last_checked;
        std::chrono::duration<double> mavros_timeout;

};

MavrosLEDController::MavrosLEDController() : 
    Node("mavros_led", "", rclcpp::NodeOptions().allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true)) 
{   
    this->parse_event_params();

    double timeout; // Get timeout parameter
    this->get_parameter_or("mavros_timeout", timeout, 30.0);
    this->mavros_timeout = chrono::duration<double>(timeout);

    this->get_parameter_or("connection_check_rate", timeout, 0.5);
    this->connection_check_rate = chrono::duration<double>(1.0/timeout);

    // Clients
    this->set_effect_client = this->create_client<clover_ros2_msgs::srv::SetLEDEffect>("set_effect");
    this->set_effect_client->wait_for_service(10s);

    // Subscribers
    this->mavros_state_sub = this->create_subscription<mavros_msgs::msg::State>(
		"mavros/state", 10,
		std::bind(&MavrosLEDController::handleMavrosState, this, std::placeholders::_1)
	);

    this->emergency_stop_sub = this->create_subscription<std_msgs::msg::Empty>(
        "/emergency_stop", 1, 
        [this](const std_msgs::msg::Empty::SharedPtr msg){(void)msg; this->emergency_stop();}
    );

    // Timers
    this->connected = false;
    this->check_timer = this->create_wall_timer(this->connection_check_rate, [this](){this->check_connection_cb();});
 
}

void MavrosLEDController::parse_event_params() {
    std::map<string, string> _event_colour_map; // Maps Mavros Event 
    this->get_parameters("events", _event_colour_map); // Params in format events.xxx.yyy

    // RCLCPP_INFO(this->get_logger(), "Parsing events");
    for (const auto& kv : _event_colour_map) {
        // RCLCPP_INFO(this->get_logger(), "events is %s: %s", kv.first.c_str(), kv.second.c_str());
        string name = "";
        string param = "";

        size_t pos = kv.first.find(".");
        if ( pos != std::string::npos) {
            // remove the part before "."
            name = kv.first.substr(0, pos); // Get first part, name
            param = kv.first.substr(pos+1); // Get second part
        }

        // Insert or get reference to effect based on name
        clover_ros2_msgs::srv::SetLEDEffect::Request::SharedPtr ledeffect;
        if(this->event_effect_map.find(name) == this->event_effect_map.end()) {
            // Name not found, create new
            ledeffect = std::make_shared<clover_ros2_msgs::srv::SetLEDEffect::Request>();
            ledeffect->effect = "fill"; // Default
            this->event_effect_map.insert(std::pair<string, clover_ros2_msgs::srv::SetLEDEffect::Request::SharedPtr>(name, ledeffect));
        } else {
            ledeffect = this->event_effect_map[name];
        }

        // RCLCPP_INFO(this->get_logger(), "Parsing name: %s, param: %s, effect: %s", name.c_str(), param.c_str(), kv.second.c_str());

        // Parse param into a SetLEDEffect service
        if(param == "effect"){ledeffect->effect = kv.second;} 
        if (param == "r") {ledeffect->r = stoi(kv.second);} 
        if (param == "b") {ledeffect->b = stoi(kv.second);}
        if (param == "g") {ledeffect->g = stoi(kv.second);} 
        if (param == "brightness") {ledeffect->brightness = stoi(kv.second);}
        if (param == "priority") {ledeffect->priority = stoi(kv.second);}
        if (param == "duration") {ledeffect->duration = stof(kv.second);}
        if (param == "base") {ledeffect->base = kv.second=="true";}

    }
    RCLCPP_INFO(this->get_logger(), "Parsed Effects from parameters:");   
    for(const auto& kv: this->event_effect_map) {
        RCLCPP_INFO(this->get_logger(), 
            "* %s %s: %s (r: %u, g: %u, b: %u, br: %u), duration: %fs, priority: %u", 
            kv.first.c_str(),
            kv.second->base?"[base]":"",
            kv.second->effect.c_str(),
            kv.second->r,
            kv.second->g,
            kv.second->b, 
            kv.second->brightness, 
            kv.second->duration,
            kv.second->priority
            );
    }

}


void MavrosLEDController::emergency_stop() {
    // Send Emergency Stop Colour if specified, will stop itself once not pressed after 1 second. 
    if(this->event_effect_map.find("emergency_stop_topic") != this->event_effect_map.end()) {
        this->apply_event_effect("emergency_stop_topic");
    } else {
        auto ledeffect = std::make_shared<clover_ros2_msgs::srv::SetLEDEffect::Request>();
        ledeffect->effect = "flash"; ledeffect->r = 255; 
        this->send_effect(ledeffect);
    }
}

void MavrosLEDController::handleMavrosState(const mavros_msgs::msg::State::SharedPtr msg)
{
    this->mavros_last_checked = this->now();

    // If previously not connected
    if(!this->connected) {
        this->connected = true;
        this->mavros_state = msg;

        // Send Base Colour if specified
        if(this->event_effect_map.find("base") != this->event_effect_map.end()) {
            this->apply_event_effect("base");
        } else {
            auto ledeffect = std::make_shared<clover_ros2_msgs::srv::SetLEDEffect::Request>();
            ledeffect->b = 255; ledeffect->base = true; ledeffect->effect = "fill";
            this->send_effect(ledeffect);
        }

        return;
    }

	if (msg->connected && !this->mavros_state->connected) {
		this->apply_event_effect("connected");
	} else if (!msg->connected && this->mavros_state->connected) {
		this->apply_event_effect("disconnected");
	} else if (msg->armed && !this->mavros_state->armed) {
		this->apply_event_effect("armed");
	} else if (!msg->armed && this->mavros_state->armed) {
		this->apply_event_effect("disarmed");
	} else if (msg->mode != this->mavros_state->mode) {
		// mode changed
		std::string mode = boost::algorithm::to_lower_copy(msg->mode);
		if (mode.find(".") != std::string::npos) {
			// remove the part before "."
			mode = mode.substr(mode.find(".") + 1);
		}
        this->apply_event_effect(mode);
	}
	this->mavros_state = msg;
}

void MavrosLEDController::apply_event_effect(const string& event) {
    // Insert or get reference to effect based on name
    clover_ros2_msgs::srv::SetLEDEffect::Request::SharedPtr ledeffect;
    if(this->event_effect_map.find(event) != this->event_effect_map.end()) {
        ledeffect = this->event_effect_map[event];
    } else {
        RCLCPP_ERROR(this->get_logger(), "Event '%s' not found in parameter list, not sending", event.c_str());
        // ledeffect = std::make_shared<clover_ros2_msgs::srv::SetLEDEffect::Request>();
        // ledeffect->effect = "fill";
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Applying event effect: %s", event.c_str());
    this->send_effect(ledeffect);
}

bool MavrosLEDController::send_effect(const clover_ros2_msgs::srv::SetLEDEffect::Request::SharedPtr effect, bool wait) {
    try{
        while (!this->set_effect_client->wait_for_service(std::chrono::duration<int>(2))) {
            if (!rclcpp::ok()) {
                throw std::runtime_error("Interrupted while waiting for set effect service. Exiting.");
            }
            RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
        }

        auto result_future = this->set_effect_client->async_send_request(effect);
        if(!wait) {
            return true;
        }

        if (result_future.wait_for(std::chrono::duration<double>(10.0)) != std::future_status::ready)
        {
            throw std::runtime_error("Set Effect Service call failed");
        }

        auto result = result_future.get();
        if(!result->success) {
            throw std::runtime_error("Set Effect Service call failed");
        }
        RCLCPP_INFO(this->get_logger(), "Message: %s", result->message.c_str());
    } catch (const std::exception& e) {
		string message = e.what();
		RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
		return false;
	}
    return true;
}

void MavrosLEDController::check_connection_cb() {
    if(this->connected){
        // Checks to do if connected
        if(this->now() - this->mavros_last_checked > this->mavros_timeout) {
            RCLCPP_WARN(this->get_logger(), "Lost MAVROS Connection");
            auto ledeffect = std::make_shared<clover_ros2_msgs::srv::SetLEDEffect::Request>();
            ledeffect->effect = "reset";
            this->send_effect(ledeffect);
            this->connected = false;
        }
    }

    if(!this->connected) {
        // If not connected
        RCLCPP_WARN(this->get_logger(), "MAVROS not connected");
        // Mavros Connection timed out, send 'no_mavros' colour
        this->apply_event_effect("no_mavros");
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MavrosLEDController>());
    rclcpp::shutdown();
    return 0;
}