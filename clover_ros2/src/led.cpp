#include <chrono>
#include <memory>
// #include <boost/algorithm/string.hpp>

#include "rclcpp/rclcpp.hpp"
#include "clover_ros2/srv/set_led_effect.hpp"
#include "led_msgs/srv/set_le_ds.hpp"
#include "led_msgs/msg/led_state.hpp"
#include "led_msgs/msg/led_state_array.hpp"

// #include "sensor_msgs/msg/battery_state.h"
//#include "mavros_msgs/msg/State.h"

using namespace std::chrono_literals;

class CloverLEDController : public rclcpp::Node
{
    public:
        CloverLEDController();
        void callSetLeds();
        void rainbow(uint8_t n, uint8_t& r, uint8_t& g, uint8_t& b);
        void fill(uint8_t r, uint8_t g, uint8_t b);
        void proceed();
        bool setEffect(std::shared_ptr<clover_ros2::srv::SetLEDEffect::Request> req, std::shared_ptr<clover_ros2::srv::SetLEDEffect::Response> res);
        void handleState(const led_msgs::msg::LEDStateArray::SharedPtr msg);
        // void notify(const std::string& event);
        // void handleMavrosState(const mavros_msgs::State& msg);

    private:
        std::shared_ptr<clover_ros2::srv::SetLEDEffect::Request> current_effect;
        int led_count;
		rclcpp::TimerBase::SharedPtr timer;
		rclcpp::Time start_time;

        std::chrono::nanoseconds blink_rate, blink_fast_rate, flash_delay, fade_period, wipe_period, rainbow_period;
        double low_battery_threshold;
        bool blink_state;
		int flash_number;
        std::shared_ptr<led_msgs::srv::SetLEDs::Request> set_leds;
        std::shared_ptr<led_msgs::msg::LEDStateArray> state, start_state;

        rclcpp::Client<led_msgs::srv::SetLEDs>::SharedPtr set_leds_srv;
        rclcpp::Subscription<led_msgs::msg::LEDStateArray>::SharedPtr state_sub;
        rclcpp::Service<clover_ros2::srv::SetLEDEffect>::SharedPtr set_effect;

        //mavros_msgs::State mavros_state;
        int counter;

		void restartTimer(double seconds);
};

CloverLEDController::CloverLEDController() : Node("led")
{
    RCLCPP_INFO(this->get_logger(), "Constructor Initialisation");
    double blink_rate, blink_fast_rate, flash_delay, fade_period, wipe_period, rainbow_period;
    this->get_parameter_or("blink_rate",blink_rate, 2.0);
	this->get_parameter_or("blink_fast_rate",blink_fast_rate, blink_rate * 2);
	this->get_parameter_or("fade_period",fade_period, 0.5);
	this->get_parameter_or("wipe_period",wipe_period, 0.5);
	this->get_parameter_or("flash_delay",flash_delay, 0.1);
	this->get_parameter_or("flash_number",this->flash_number, 3);
	this->get_parameter_or("rainbow_period",rainbow_period, 5.0);
	this->get_parameter_or("notify/low_battery/threshold", this->low_battery_threshold, 3.7);

	// Cast rate parameters to nanoseconds
    this->blink_rate =      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(blink_rate));
    this->blink_fast_rate = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(blink_fast_rate));
    this->fade_period =     std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(fade_period));
    this->wipe_period =     std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(wipe_period));
    this->flash_delay = 	std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(flash_delay));
    this->rainbow_period =  std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(rainbow_period));
    
    RCLCPP_INFO(this->get_logger(), "Set Parameters");

	// Initialise set leds
	this->state = std::make_shared<led_msgs::msg::LEDStateArray>();
	this->start_state = std::make_shared<led_msgs::msg::LEDStateArray>();
	this->set_leds = std::make_shared<led_msgs::srv::SetLEDs::Request>();
	this->current_effect = std::make_shared<clover_ros2::srv::SetLEDEffect::Request>();
	this->current_effect->effect = "None";
	this->current_effect->r = 0;
	this->current_effect->g = 0;
	this->current_effect->b = 0;

    // First need to wait for service
    // ros::service::waitForService("set_leds"); // cannot work without set_leds service
    this->set_leds_srv = this->create_client<led_msgs::srv::SetLEDs>("set_leds");
    this->set_leds_srv->wait_for_service(10s);

    this->state_sub = this->create_subscription<led_msgs::msg::LEDStateArray>(
        "state", 10,
        std::bind(&CloverLEDController::handleState, this, std::placeholders::_1)
    );
    this->set_effect = this->create_service<clover_ros2::srv::SetLEDEffect>(
        "set_effect",
        std::bind(&CloverLEDController::setEffect, this, std::placeholders::_1, std::placeholders::_2)
    );
    RCLCPP_INFO(this->get_logger(), "Create Service");

	this->restartTimer(1.0);
	// this->timer = this->create_wall_timer(
	// 	0s, std::bind(&CloverLEDController::proceed, this));
    RCLCPP_INFO(this->get_logger(), "End Constructor");
}

void CloverLEDController::restartTimer(double seconds) 
{
	if (this->timer) {
		this->timer->cancel();
	}
	this->timer = this->create_wall_timer(
		std::chrono::duration<double>(seconds), 
		std::bind(&CloverLEDController::proceed, this)
	);
    RCLCPP_INFO(this->get_logger(), "Reset Timer to %f", seconds);
}

void CloverLEDController::callSetLeds()
{
    RCLCPP_INFO(this->get_logger(), "Sending LED Call request");
	this->set_leds_srv->async_send_request(this->set_leds);
}

void CloverLEDController::rainbow(uint8_t n, uint8_t& r, uint8_t& g, uint8_t& b)
{
	if (n < 255 / 3) {
		r = n * 3;
		g = 255 - n * 3;
		b = 0;
	} else if (n < 255 / 3 * 2) {
		n -= 255 / 3;
		r = 255 - n * 3;
		g = 0;
		b = n * 3;
	} else {
		n -= 255 / 3 * 2;
		r = 0;
		g = n * 3;
		b = 255 - n * 3;
	}
}

void CloverLEDController::fill(uint8_t r, uint8_t g, uint8_t b)
{
	this->set_leds->leds.resize(this->led_count);
	for (int i = 0; i < led_count; i++) {
		this->set_leds->leds[i].index = i;
		this->set_leds->leds[i].r = r;
		this->set_leds->leds[i].g = g;
		this->set_leds->leds[i].b = b;
	}
	this->callSetLeds();
}

void CloverLEDController::handleState(const led_msgs::msg::LEDStateArray::SharedPtr msg)
{
    this->state = msg;
    this->led_count = this->state->leds.size();
    // RCLCPP_INFO(this->get_logger(), "Handling received led state with %d leds", this->led_count);
}

void CloverLEDController::proceed()
{
	this->counter++;
	uint8_t r, g, b;
	this->set_leds->leds.clear();
	this->set_leds->leds.resize(this->led_count);

	if (this->current_effect->effect == "blink" || this->current_effect->effect == "blink_fast") {
		RCLCPP_INFO(this->get_logger(), "proceed: blinking");
		this->blink_state = !this->blink_state;
		// toggle all leds
		if (this->blink_state) {
			this->fill(this->current_effect->r, this->current_effect->g, this->current_effect->b);
		} else {
			this->fill(0, 0, 0);
		}

	} 
	else if (this->current_effect->effect == "fade") {
		RCLCPP_INFO(this->get_logger(), "proceed: fade");
	// 	// fade all leds from starting state
		double time_elapsed = (double) (this->now() - this->start_time).nanoseconds();
		double passed = std::min( time_elapsed / this->fade_period.count(), 1.0);
		double one_minus_passed = 1 - passed;
		for (int i = 0; i < this->led_count; i++) {
			this->set_leds->leds[i].index = i;
			this->set_leds->leds[i].r = one_minus_passed * this->start_state->leds[i].r + passed * this->current_effect->r;
			this->set_leds->leds[i].g = one_minus_passed * this->start_state->leds[i].g + passed * this->current_effect->g;
			this->set_leds->leds[i].b = one_minus_passed * this->start_state->leds[i].b + passed * this->current_effect->b;
		}
		this->callSetLeds();
		if (passed >= 1.0) {
			// fade finished
			this->timer->cancel();
		}
	} 
	else if (this->current_effect->effect == "wipe") {
		RCLCPP_INFO(this->get_logger(), "proceed: wipe");
		this->set_leds->leds.resize(1);
		this->set_leds->leds[0].index = this->counter - 1;
		this->set_leds->leds[0].r = this->current_effect->r;
		this->set_leds->leds[0].g = this->current_effect->g;
		this->set_leds->leds[0].b = this->current_effect->b;
		this->callSetLeds();
		if (this->counter == this->led_count) {
			// wipe finished
			this->timer->cancel();
		}

	} 
	else if (this->current_effect->effect == "rainbow_fill") {
		RCLCPP_INFO(this->get_logger(), "proceed: rainbow_fill");
		this->rainbow(this->counter % 255, r, g, b);
		for (int i = 0; i < this->led_count; i++) {
			this->set_leds->leds[i].index = i;
			this->set_leds->leds[i].r = r;
			this->set_leds->leds[i].g = g;
			this->set_leds->leds[i].b = b;
		}
		this->callSetLeds();
	}
	else if (this->current_effect->effect == "rainbow") {
		RCLCPP_INFO(this->get_logger(), "proceed: rainbow");
		for (int i = 0; i < this->led_count; i++) {
			int pos = (int)round(this->counter + (255.0 * i / this->led_count)) % 255;
			this->rainbow(pos % 255, r, g, b);
			this->set_leds->leds[i].index = i;
			this->set_leds->leds[i].r = r;
			this->set_leds->leds[i].g = g;
			this->set_leds->leds[i].b = b;
		}
		this->callSetLeds();
	}
}


bool CloverLEDController::setEffect(std::shared_ptr<clover_ros2::srv::SetLEDEffect::Request> req, std::shared_ptr<clover_ros2::srv::SetLEDEffect::Response> res)
{
    res->success = true;

	RCLCPP_INFO(this->get_logger(), "Received Set request for effect: %s", req->effect);

	if (req->effect == "") {
		req->effect = "fill";
	}

	if (req->effect != "flash" && req->effect != "fill" && this->current_effect->effect == req->effect &&
	    this->current_effect->r == req->r && this->current_effect->g == req->g && this->current_effect->b == req->b) {
		res->message = "Effect already set, skip";
		return true;
	}

	if (req->effect == "fill") {
		this->fill(req->r, req->g, req->b);

	} else if (req->effect == "blink") {
		this->restartTimer(1.0/this->blink_rate.count());

	} else if (req->effect == "blink_fast") {
		this->restartTimer(1.0/this->blink_fast_rate.count());

	} else if (req->effect == "fade") {
		this->restartTimer(0.05);

	} else if (req->effect == "wipe") {
		this->restartTimer((double) this->wipe_period.count()/(double) this->led_count);

	} else if (req->effect == "flash") {
		for(int i = 0; i < this->flash_number; i++){
			this->fill(req->r, req->g, req->b);
			rclcpp::sleep_for(this->flash_delay);
			this->fill(0, 0, 0);
			rclcpp::sleep_for(this->flash_delay);
		}
		if (this->current_effect->effect == "fill"||
		    this->current_effect->effect == "fade" ||
		    this->current_effect->effect == "wipe") {
			// restore previous filling
			for (int i = 0; i < led_count; i++) {
				this->fill(this->current_effect->r, this->current_effect->g, this->current_effect->b);
			}
			callSetLeds();
		}
		return true; // this effect happens only once

	} else if (req->effect == "rainbow_fill") {
		this->restartTimer((double) this->rainbow_period.count()/255.0);

	} else if (req->effect == "rainbow") {
		this->restartTimer((double) this->rainbow_period.count()/255.0);

	} else {
		res->message = "Unknown effect: " + req->effect + ". Available effects are fill, fade, wipe, blink, blink_fast, flash, rainbow, rainbow_fill.";
		RCLCPP_ERROR(this->get_logger(), "%s", res->message.c_str());
		res->success = false;
		return false;
	}

	// set current effect
	this->current_effect = req;
	this->counter = 0;
	this->start_state = this->state;
	this->start_time = this->now();

	return true;
}



int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CloverLEDController>());
    rclcpp::shutdown();
    return 0;
}
