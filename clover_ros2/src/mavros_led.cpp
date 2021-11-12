#include <chrono>
#include <memory>
#include <boost/algorithm/string.hpp>

#include "rclcpp/rclcpp.hpp"
#include "clover_ros2/srv/set_led_effect.hpp"

// #include "sensor_msgs/msg/battery_state.h"
#include "mavros_msgs/msg/state.hpp"

using namespace std;

class MavrosLEDController: public rclcpp::Node
{
    public:
        MavrosLEDController();

    private:
        void handleMavrosState(const mavros_msgs::msg::State::SharedPtr msg);
        void notify(const std::string& event);

        void parse_event_params();


        // Parameters
        std::map<string, clover_ros2::srv::SetLEDEffect::Request::SharedPtr> event_effect_map; // Maps Mavros Event 

        // Set Effect Service Client
        rclcpp::Client<clover_ros2::srv::SetLEDEffect>::SharedPtr set_effect_client;

        // Mavros State
        rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr mavros_state_sub;
        std::shared_ptr<mavros_msgs::msg::State> mavros_state;

};

MavrosLEDController::MavrosLEDController() : 
    Node("mavros_led", "", rclcpp::NodeOptions().allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true)) 
{   
    this->parse_event_params();

    // Mavros state
    this->mavros_state_sub = this->create_subscription<mavros_msgs::msg::State>(
		"mavros/state", 10,
		std::bind(&MavrosLEDController::handleMavrosState, this, std::placeholders::_1)
	);
}

void MavrosLEDController::parse_event_params() {
    std::map<string, string> _event_colour_map; // Maps Mavros Event 
    this->get_parameters("events", _event_colour_map); // Params in format events.xxx.yyy

    RCLCPP_INFO(this->get_logger(), "Parsing events:");
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
        RCLCPP_INFO(this->get_logger(), "Loaded event '%s', param as %s, value: %s", name.c_str(), param.c_str(), kv.second.c_str());

        // Insert or get reference to effect based on name
        clover_ros2::srv::SetLEDEffect::Request::SharedPtr ledeffect;
        if(this->event_effect_map.find(name) == this->event_effect_map.end()) {
            // Name not found, create new
            ledeffect = std::make_shared<clover_ros2::srv::SetLEDEffect::Request>();
            this->event_effect_map.insert(std::pair<string, clover_ros2::srv::SetLEDEffect::Request::SharedPtr>(name, ledeffect));
        } else {
            ledeffect = this->event_effect_map[name];
        }

        // Parse param into a SetLEDEffect service
        if(param == "effect"){
            ledeffect->effect = kv.second;
        } else if (param == "r") {
            ledeffect->r = stoi(kv.second);
        } else if (param == "b") {
            ledeffect->b = stoi(kv.second);
        } else if (param == "g") {
            ledeffect->g = stoi(kv.second);
        } else if (param == "brightness") {
            ledeffect->brightness = stoi(kv.second);
        } else if (param == "priority") {
            ledeffect->priority = stoi(kv.second);
        } else if (param == "duration") {
            ledeffect->duration = stof(kv.second);
        }

    }
    RCLCPP_INFO(this->get_logger(), "Finished Parsing Parameters");   
}

void MavrosLEDController::handleMavrosState(const mavros_msgs::msg::State::SharedPtr msg)
{
	if (!this->mavros_state){
		this->mavros_state = msg;
		return;
	}

	if (msg->connected && !this->mavros_state->connected) {
		// notify("connected");
	} else if (!msg->connected && this->mavros_state->connected) {
		// notify("disconnected");
	} else if (msg->armed && !this->mavros_state->armed) {
		// notify("armed");
	} else if (!msg->armed && this->mavros_state->armed) {
		// notify("disarmed");
	} else if (msg->mode != this->mavros_state->mode) {
		// mode changed
		std::string mode = boost::algorithm::to_lower_copy(msg->mode);
		if (mode.find(".") != std::string::npos) {
			// remove the part before "."
			mode = mode.substr(mode.find(".") + 1);
		}
		// std::string err;
		// if (ros::names::validate(mode, err)) {
		// this->notify(mode);
		// }
	}
	this->mavros_state = msg;
}

void MavrosLEDController::notify(const std::string& event)
{

}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MavrosLEDController>());
    rclcpp::shutdown();
    return 0;
}