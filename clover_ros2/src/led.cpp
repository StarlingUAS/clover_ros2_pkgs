#include <chrono>
#include <memory>
#include <vector>
#include <algorithm>
#include <boost/algorithm/string.hpp>

#include "rclcpp/rclcpp.hpp"
#include "clover_ros2/srv/set_led_effect.hpp"
#include "led_msgs/srv/set_le_ds.hpp"
#include "led_msgs/msg/led_state.hpp"
#include "led_msgs/msg/led_state_array.hpp"

// #include "sensor_msgs/msg/battery_state.h"
#include "mavros_msgs/msg/state.hpp"

using namespace std::chrono_literals;

const std::vector<std::string> VALID_EFFECTS = {
	"fill",
	"blink",
	"blink_fast",
	"fade",
	"wipe",
	"flash",
	"rainbow_fill",
	"rainbow"
};

class Effect {
	public:
		Effect(std::shared_ptr<clover_ros2::srv::SetLEDEffect::Request> effect, rclcpp::Time curr_time)
			:_effect(effect) {this->combine(std::make_shared<Effect>(effect), curr_time);}

		Effect(std::shared_ptr<clover_ros2::srv::SetLEDEffect::Request> effect, bool infinite)
			:_effect(effect), _infinite(infinite){}

		Effect(std::shared_ptr<clover_ros2::srv::SetLEDEffect::Request> effect)
			:_effect(effect), _infinite(true){}

		bool operator<(const Effect& other) {
			return this->_effect->priority < other._effect->priority;
		}

		bool operator==(const Effect& other) {
			return this->_effect->effect == other._effect->effect &&
				   this->_effect->r == other._effect->r &&
				   this->_effect->g == other._effect->g &&
				   this->_effect->b == other._effect->b &&
				   this->_effect->priority == other._effect->priority;
		}

		void combine(const std::shared_ptr<Effect> other, rclcpp::Time curr_time) {
			double duration = other->_effect->duration;
			this->_effect = other->_effect;
			if(duration == 0) {
				this->_infinite = true;
			} else {
				this->_end_time = curr_time + std::chrono::duration<double>(duration);
			}
		}

		std::shared_ptr<clover_ros2::srv::SetLEDEffect::Request> get_effect() {
			return this->_effect;
		}

		bool finished(rclcpp::Time curr_time) {
			return !this->_infinite && curr_time > this->_end_time;
		}

		private:
			std::shared_ptr<clover_ros2::srv::SetLEDEffect::Request> _effect;
			rclcpp::Time _end_time;
			bool _infinite = false;
};

class CloverLEDController : public rclcpp::Node
{
    public:
        CloverLEDController();
		~CloverLEDController();
        void callSetLeds();
        void rainbow(uint8_t n, uint8_t& r, uint8_t& g, uint8_t& b);
        void fill(uint8_t r, uint8_t g, uint8_t b);
		void set_leds_index(uint8_t i, uint8_t index, uint8_t r, uint8_t g, uint8_t b);
        void proceed();
        bool setEffect(std::shared_ptr<clover_ros2::srv::SetLEDEffect::Request> req, std::shared_ptr<clover_ros2::srv::SetLEDEffect::Response> res);
		void setEffectRaw(std::string eff, int r, int g, int b, float brightness=255.0, float duration=-1.0, bool notify=false);
        void handleState(const led_msgs::msg::LEDStateArray::SharedPtr msg);
		bool startEffect(std::shared_ptr<Effect> effect);

    private:
		std::shared_ptr<Effect> base_effect;
		std::vector<std::shared_ptr<Effect>> pq;
		
		std::shared_ptr<Effect> curr_effect;
        std::shared_ptr<clover_ros2::srv::SetLEDEffect::Request> current_effect;
        int led_count;

		rclcpp::TimerBase::SharedPtr timer;
		rclcpp::Time start_time;

        double blink_rate, blink_fast_rate, brightness, flash_delay, fade_period, wipe_period, rainbow_period;
        double low_battery_threshold;
        bool blink_state;
		int flash_number;

		bool notify_state;

		uint32_t num_priority_levels;

        std::shared_ptr<led_msgs::srv::SetLEDs::Request> set_leds;
        std::shared_ptr<led_msgs::msg::LEDStateArray> state, start_state;

        rclcpp::Client<led_msgs::srv::SetLEDs>::SharedPtr set_leds_srv;
        rclcpp::Subscription<led_msgs::msg::LEDStateArray>::SharedPtr state_sub;
        rclcpp::Service<clover_ros2::srv::SetLEDEffect>::SharedPtr set_effect;

		rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr mavros_state_sub;
        std::shared_ptr<mavros_msgs::msg::State> mavros_state;
        int counter;

		void restartTimer(double seconds);
};

CloverLEDController::CloverLEDController() : 
	Node("led", 
		 "",
		 rclcpp::NodeOptions()
			.allow_undeclared_parameters(true)
			.automatically_declare_parameters_from_overrides(true)
	)
{
	// this->get_node_options().allow_undeclared_parameters(true);
	// this->get_node_options().automatically_declare_parameters_from_overrides(true);
    // double blink_rate, blink_fast_rate, flash_delay, fade_period, wipe_period, rainbow_period;
    this->get_parameter_or("blink_rate",this->blink_rate, 2.0);
	this->get_parameter_or("blink_fast_rate",this->blink_fast_rate, blink_rate * 2.0);
	this->get_parameter_or("fade_period",this->fade_period, 0.5);
	this->get_parameter_or("wipe_period",this->wipe_period, 0.5);
	this->get_parameter_or("flash_delay",this->flash_delay, 0.1);
	this->get_parameter_or("flash_number",this->flash_number, 1);
	this->get_parameter_or("rainbow_period",this->rainbow_period, 5.0);
	this->get_parameter_or("brightness", this->brightness, 64.0);
		// res->message = "Effect already set, skip";
	this->get_parameter_or("notify/low_battery/threshold", this->low_battery_threshold, 3.7);
	this->get_parameter_or("num_priority_levels", this->num_priority_levels, 9U);

	// Initialise set leds
	this->state = std::make_shared<led_msgs::msg::LEDStateArray>();
	this->start_state = std::make_shared<led_msgs::msg::LEDStateArray>();
	this->set_leds = std::make_shared<led_msgs::srv::SetLEDs::Request>();
	this->current_effect = std::make_shared<clover_ros2::srv::SetLEDEffect::Request>();
	this->current_effect->effect = "";
	this->current_effect->r = 0;
	this->current_effect->g = 0;
	this->current_effect->b = 0;

	// Resize Queue with all initialised to nullptr
	this->pq.resize(this->num_priority_levels, nullptr);

	// New values
	this->base_effect = std::make_shared<Effect>(std::make_shared<clover_ros2::srv::SetLEDEffect::Request>());

    // First need to wait for service
    this->set_leds_srv = this->create_client<led_msgs::srv::SetLEDs>("set_leds");
    this->set_leds_srv->wait_for_service(10s);

    this->state_sub = this->create_subscription<led_msgs::msg::LEDStateArray>(
        "led_state", 10,
        std::bind(&CloverLEDController::handleState, this, std::placeholders::_1)
    );

    this->set_effect = this->create_service<clover_ros2::srv::SetLEDEffect>(
        "set_effect",
        std::bind(&CloverLEDController::setEffect, this, std::placeholders::_1, std::placeholders::_2)
    );

	this->restartTimer(0.5);
	RCLCPP_INFO(this->get_logger(), "LED Effects Controller Initialised");
}

CloverLEDController::~CloverLEDController() {
	// Destructor simply turns off lights.
	this->setEffectRaw("", 0, 0, 0);
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
    RCLCPP_DEBUG(this->get_logger(), "Reset Timer to %f", seconds);
}

void CloverLEDController::callSetLeds()
{
    // RCLCPP_INFO(this->get_logger(), "Sending LED Call request");
	this->set_leds_srv->async_send_request(this->set_leds);
}

void CloverLEDController::set_leds_index(uint8_t i, uint8_t index, uint8_t r, uint8_t g, uint8_t b)
{
	this->set_leds->leds[i].index = index;
	this->set_leds->leds[i].r = r;
	this->set_leds->leds[i].g = g;
	this->set_leds->leds[i].b = b;
	this->set_leds->leds[i].brightness = this->brightness;
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
		this->set_leds_index(
			i, i, r, g, b
		);
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
	rclcpp::Time curr_time = this->now();

	// Check for current most prioritised effect
	uint8_t p_idx = this->num_priority_levels - 1;
	std::shared_ptr<Effect> effect;
	for(auto it = this->pq.rbegin(); it != this->pq.rend(); ++it ) {
		if(*it){effect = *it; break;}
		p_idx--;
	}

	// Check if effect is found
	if(effect) {
		// Check if effect has expired, remove the effect from the queue
		if(effect->finished(curr_time)) {
			this->pq[p_idx] = nullptr;
		}
	} else {
		// If no effect found, set effect to base effect
		effect = this->base_effect;
	}

	// Check if effect is different from current effect
	if(effect != this->curr_effect) {
		// Parse and set state for new effect
		this->startEffect(effect);
	}

	// Execute effect
	this->current_effect = this->curr_effect->get_effect();
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
		RCLCPP_DEBUG(this->get_logger(), "proceed: fade");
	// 	// fade all leds from starting state
		double time_elapsed = (double) (this->now() - this->start_time).nanoseconds();
		double passed = std::min( time_elapsed / this->fade_period, 1.0);
		double one_minus_passed = 1 - passed;
		for (int i = 0; i < this->led_count; i++) {
			this->set_leds_index(
				i, i,
				one_minus_passed * this->start_state->leds[i].r + passed * this->current_effect->r,
				one_minus_passed * this->start_state->leds[i].g + passed * this->current_effect->g,
				one_minus_passed * this->start_state->leds[i].b + passed * this->current_effect->b
			);
		}
		this->callSetLeds();
		if (passed >= 1.0) {
			// fade finished
			this->timer->cancel();
		}
	} 
	else if (this->current_effect->effect == "wipe") {
		RCLCPP_DEBUG(this->get_logger(), "proceed: wipe");
		this->set_leds->leds.resize(1);
		this->set_leds_index(
			0,
			this->counter - 1,
			this->current_effect->r,
			this->current_effect->g,
			this->current_effect->b
		);
		this->callSetLeds();
		if (this->counter == this->led_count) {
			// wipe finished
			this->timer->cancel();
		}

	} 
	else if (this->current_effect->effect == "rainbow_fill") {
		RCLCPP_DEBUG(this->get_logger(), "proceed: rainbow_fill");
		this->rainbow(this->counter % 255, r, g, b);
		for (int i = 0; i < this->led_count; i++) {
			this->set_leds_index(i,i,r,g,b);
		}
		this->callSetLeds();
	}
	else if (this->current_effect->effect == "rainbow") {
		RCLCPP_DEBUG(this->get_logger(), "proceed: rainbow");
		for (int i = 0; i < this->led_count; i++) {
			int pos = (int)round(this->counter + (255.0 * i / this->led_count)) % 255;
			this->rainbow(pos % 255, r, g, b);
			this->set_leds_index(i,i,r,g,b);
		}
		this->callSetLeds();
	}
}

bool CloverLEDController::setEffect(std::shared_ptr<clover_ros2::srv::SetLEDEffect::Request> req, std::shared_ptr<clover_ros2::srv::SetLEDEffect::Response> res)
{
    res->success = true;

	RCLCPP_INFO(
		this->get_logger(), 
		"Received led set effect: %s (r: %i, g: %i, b: %i) brightness: %i, duration: %f, notify: %s", 
		req->effect.c_str(), req->r, req->g, req->b, req->brightness, req->duration, req->notify ? "true" : "false"
	);

	if(!req->priority) {
		req->priority = 0;
	}

	if(req->priority > this->num_priority_levels) {
		res->message = "Requested priority is greater than allowed priority";
		res->success = false;
		return false;
	}

	// If 'base' set, set base and clear queue
	if(req->base || req->effect=="reset") {
		if(req->base){this->base_effect = std::make_shared<Effect>(req);}
		std::fill(this->pq.begin(), this->pq.end(), nullptr); // Probably should lock this...
		res->message = "Queue emptied";
		return true;
	}

	// Set Defaults
	if (req->effect == "") {req->effect = "fill";}
	if (!req->brightness) {this->brightness = req->brightness;}

	// Check valid effect first
	bool found = (std::find(VALID_EFFECTS.begin(), VALID_EFFECTS.end(), req->effect) != VALID_EFFECTS.end());
	if(!found) {
		res->message = "Unknown effect: " + req->effect + ". Available effects are fill, fade, wipe, blink, blink_fast, flash, rainbow, rainbow_fill.";
		RCLCPP_ERROR(this->get_logger(), "%s", res->message.c_str());
		return false;
	}

	// Overwrite the effect at priority or add to priority list. 
	rclcpp::Time curr_time;
	std::shared_ptr<Effect> new_effect = std::make_shared<Effect>(req, this->now());
	this->pq[req->priority] = new_effect; 
		
	return true;
}


bool CloverLEDController::startEffect(std::shared_ptr<Effect> effect){

	auto req = effect->get_effect();

	///////////////
	// Process effect
	if (req->effect != "flash" && req->effect != "fill" && this->current_effect->effect == req->effect &&
	    this->current_effect->r == req->r && this->current_effect->g == req->g && this->current_effect->b == req->b) {
		return true;
	}

	if (req->effect == "fill") {
		this->fill(req->r, req->g, req->b);

	} else if (req->effect == "blink") {
		this->restartTimer(1.0/this->blink_rate);

	} else if (req->effect == "blink_fast") {
		this->restartTimer(1.0/this->blink_fast_rate);

	} else if (req->effect == "fade") {
		this->restartTimer(0.05);

	} else if (req->effect == "wipe") {
		this->restartTimer((double) this->wipe_period/(double) this->led_count);

	} else if (req->effect == "flash") {
		auto _flash_delay = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(this->flash_delay));
		for(int i = 0; i < this->flash_number; i++){
			this->fill(req->r, req->g, req->b);
			rclcpp::sleep_for(_flash_delay);
			this->fill(0, 0, 0);
			rclcpp::sleep_for(_flash_delay);
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
		this->restartTimer(this->rainbow_period/255.0);

	} else if (req->effect == "rainbow") {
		this->restartTimer(this->rainbow_period/255.0);

	} else {
		RCLCPP_ERROR(this->get_logger(), "Should never get here");
		return false;
	}
	// finish processing effects request
	///////////////

	// set current effect
	this->curr_effect = effect;
	this->counter = 0;
	this->start_state = this->state;
	this->start_time = this->now();

	return true;
}

void CloverLEDController::setEffectRaw(std::string eff, int b, int g, int r, float brightness, float duration, bool notify)
{
	auto effect = std::make_shared<clover_ros2::srv::SetLEDEffect::Request>();
	effect->effect = eff;
	effect->r = r;
	effect->g = g;
	effect->b = b;

	effect->brightness = brightness;

	if(duration > 0.0) {
		effect->duration = duration;
	}
	if(notify) {
		effect->notify = notify;
	}
	this->setEffect(effect, std::make_shared<clover_ros2::srv::SetLEDEffect::Response>());
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CloverLEDController>());
    rclcpp::shutdown();
    return 0;
}
