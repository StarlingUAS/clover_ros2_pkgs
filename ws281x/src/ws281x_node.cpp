#include "rclcpp/rclcpp.hpp"
#include "rclcpp/utilities.hpp"

// #include "led_msgs/msg/LEDStateArray.hpp"
#include "led_msgs/msg/led_state_array.hpp"
#include "led_msgs/srv/set_le_ds.hpp"
#include "ws281x/srv/set_gamma.hpp"

// #include <led_msgs/SetLEDs.h>
// #include <led_msgs/LEDStateArray.h>
// #include <ws281x/SetGamma.h>

#include <ws2811.h>
// #include <ros/console.h>
#include <unordered_map>
#include <signal.h>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <atomic>
#include <unistd.h>

using namespace std::chrono_literals;

constexpr uint32_t LED_RED_SHIFT   = 16;
constexpr uint32_t LED_GREEN_SHIFT = 8;
constexpr uint32_t LED_BLUE_SHIFT  = 0;
constexpr uint32_t LED_WHITE_SHIFT = 24;

constexpr uint32_t LED_RED_MASK   = (0xFF << LED_RED_SHIFT);
constexpr uint32_t LED_GREEN_MASK = (0xFF << LED_GREEN_SHIFT);
constexpr uint32_t LED_BLUE_MASK  = (0xFF << LED_BLUE_SHIFT);
constexpr uint32_t LED_WHITE_MASK = (0xFF << LED_WHITE_SHIFT);

constexpr uint32_t LED_RED   = (0x01 << LED_RED_SHIFT);
constexpr uint32_t LED_GREEN = (0x01 << LED_GREEN_SHIFT);
constexpr uint32_t LED_BLUE  = (0x01 << LED_BLUE_SHIFT);
constexpr uint32_t LED_WHITE = (0x01 << LED_WHITE_SHIFT);



std::unordered_map<std::string, uint64_t> ws2811_types = {
	{"SK6812_STRIP_RGBW", SK6812_STRIP_RGBW},
	{"SK6812_STRIP_RBGW", SK6812_STRIP_RBGW},
	{"SK6812_STRIP_GRBW", SK6812_STRIP_GRBW},
	{"SK6812_STRIP_GBRW", SK6812_STRIP_GBRW},
	{"SK6812_STRIP_BRGW", SK6812_STRIP_BRGW},
	{"SK6812_STRIP_BGRW", SK6812_STRIP_BGRW},
	{"WS2811_STRIP_RGB", WS2811_STRIP_RGB},
	{"WS2811_STRIP_RBG", WS2811_STRIP_RBG},
	{"WS2811_STRIP_GRB", WS2811_STRIP_GRB},
	{"WS2811_STRIP_GBR", WS2811_STRIP_GBR},
	{"WS2811_STRIP_BRG", WS2811_STRIP_BRG},
	{"WS2811_STRIP_BGR", WS2811_STRIP_BGR},
	{"WS2812_STRIP", WS2812_STRIP},
	{"SK6812_STRIP", SK6812_STRIP},
	{"SK6812W_STRIP", SK6812W_STRIP}
};


class LEDControl : public rclcpp::Node
{
	public:
		LEDControl();
		// ~LEDControl();
		void publishLedState();
		bool setGamma(const std::shared_ptr<ws281x::srv::SetGamma::Request> req, std::shared_ptr<ws281x::srv::SetGamma::Response> resp);
		bool setLeds(const std::shared_ptr<led_msgs::srv::SetLEDs::Request> req, std::shared_ptr<led_msgs::srv::SetLEDs::Response> resp);
		void cleanup();

	private:
		ws2811_t led_string;
		rclcpp::Publisher<led_msgs::msg::LEDStateArray>::SharedPtr led_state_pub;
		led_msgs::msg::LEDStateArray strip_state;

		rclcpp::Service<ws281x::srv::SetGamma>::SharedPtr srv_gamma;
		rclcpp::Service<led_msgs::srv::SetLEDs>::SharedPtr srv_leds;

		int param_freq;
		int param_pin;
		int param_dma;
		uint64_t param_strip_type;
		int param_led_count;
		bool param_invert;
		int param_brightness;

		rclcpp::TimerBase::SharedPtr publish_led_state_timer;
};

LEDControl::LEDControl() : Node("ws281x")
{	
	this->get_parameter_or("target_frequency", this->param_freq, WS2811_TARGET_FREQ);
	this->get_parameter_or("gpio_pin", this->param_pin, 21);
	this->get_parameter_or("dma", this->param_dma, 10);

	std::string strip_type_str;
	this->get_parameter_or("strip_type", strip_type_str, std::string("WS2811_STRIP_GBR"));
	this->get_parameter_or("led_count", this->param_led_count, 30);
	this->get_parameter_or("invert", this->param_invert, false);
	this->get_parameter_or("brightness", this->param_brightness, 255);


	auto strip_type_it = ws2811_types.find(strip_type_str);
	if (strip_type_it != ws2811_types.end()) {
		this->param_strip_type = strip_type_it->second;
	} else {
		RCLCPP_WARN(this->get_logger(), "[ws281x] Unknown strip type: %s", strip_type_str.c_str());
		this->param_strip_type = WS2811_STRIP_GBR;
	}

	if (param_freq < 0) {
		RCLCPP_WARN(this->get_logger(), "[ws281x] Target_frequency out of range, resetting to default");
		this->led_string.freq = (uint32_t)WS2811_TARGET_FREQ;
	} else {
		this->led_string.freq = (uint32_t)this->param_freq;
	}

	this->led_string.dmanum = this->param_dma;
	this->led_string.channel[0].gpionum = this->param_pin;
	this->led_string.channel[0].count = this->param_led_count;
	this->led_string.channel[0].invert = this->param_invert ? (1) : (0);
	this->led_string.channel[0].brightness = (uint8_t)this->param_brightness;
	this->led_string.channel[0].strip_type = (int)this->param_strip_type;

	// Disable second channel for now
	this->led_string.channel[1].gpionum = 0;
	this->led_string.channel[1].count = 0;
	this->led_string.channel[1].invert = 0;
	this->led_string.channel[1].brightness = 0;

	ws2811_return_t ret;
	if ((ret = ws2811_init(&(this->led_string))) != WS2811_SUCCESS) {
		RCLCPP_FATAL(this->get_logger(), "[ws281x] native library init failed: %s", ws2811_get_return_t_str(ret));
		exit(1);
	}

	rclcpp::on_shutdown(std::bind(&LEDControl::cleanup, this));

	// signal(SIGINT, &LEDControl::cleanup);
	// signal(SIGTERM, &LEDControl::cleanup);

	this->strip_state.leds.resize(this->param_led_count);

	this->srv_gamma = this->create_service<ws281x::srv::SetGamma>(
		"set_gamma", 
		std::bind(&LEDControl::setGamma, this, std::placeholders::_1, std::placeholders::_2)
	);

	this->srv_leds = this->create_service<led_msgs::srv::SetLEDs>(
		"set_leds", 
		std::bind(&LEDControl::setLeds, this, std::placeholders::_1, std::placeholders::_2)
	);

	this->led_state_pub = this->create_publisher<led_msgs::msg::LEDStateArray>("state", 1);

	this->publish_led_state_timer = this->create_wall_timer(
		1s,
		std::bind(&LEDControl::publishLedState, this)
	);
	// this->publishLedState();
}


void LEDControl::publishLedState()
{
	for(size_t i = 0; i < this->strip_state.leds.size(); ++i) {
		this->strip_state.leds[i].index = i;
		this->strip_state.leds[i].r = (this->led_string.channel[0].leds[i] & LED_RED_MASK) >> LED_RED_SHIFT;
		this->strip_state.leds[i].g = (this->led_string.channel[0].leds[i] & LED_GREEN_MASK) >> LED_GREEN_SHIFT;
		this->strip_state.leds[i].b = (this->led_string.channel[0].leds[i] & LED_BLUE_MASK) >> LED_BLUE_SHIFT;
		// led_state.w = (led_string.channel[0].leds[i] & LED_WHITE_MASK) >> LED_WHITE_SHIFT;
	}

	this->led_state_pub->publish(this->strip_state);
}

bool LEDControl::setGamma(const std::shared_ptr<ws281x::srv::SetGamma::Request> req, std::shared_ptr<ws281x::srv::SetGamma::Response> resp)
{
	for(int i = 0; i < 255; ++i) {
		this->led_string.channel[0].gamma[i] = req->gamma[i];
	}
	resp->success = 1;
	return true;
}

bool LEDControl::setLeds(const std::shared_ptr<led_msgs::srv::SetLEDs::Request> req, std::shared_ptr<led_msgs::srv::SetLEDs::Response> resp)
{
	// check validness
	for(auto const& led : req->leds) {
		if (led.index < 0 || led.index >= this->strip_state.leds.size()) {
			RCLCPP_ERROR(this->get_logger(), "[ws281x] LED index out of bounds: %u", led.index);
			resp->message = "LED index out of bounds: " + std::to_string(led.index);
			return true;
		}
	}

	for(auto const& led : req->leds) {
		auto color = uint32_t(
			LED_RED * int(led.r) +  // Red channel mask
			LED_GREEN * int(led.g) +  // Green channel mask
			LED_BLUE * int(led.b));  // Blue channel mask
		this->led_string.channel[0].leds[led.index] = color;
	}
	ws2811_return_t ret;
	if ((ret = ws2811_render(&(this->led_string))) != WS2811_SUCCESS) {
		resp->message = ws2811_get_return_t_str(ret);
		RCLCPP_ERROR(this->get_logger(), "[ws281x] Could not set LED colors: %s", resp->message.c_str());
		resp->success = false;
	} else {
		resp->success = true;
		resp->message = "";
	}
	this->publishLedState();
	return true;
}

// LEDControl::~LEDControl() //signal handling fails inside classes
void LEDControl::cleanup()
{
	for(int i = 0; i < this->led_string.channel[0].count; ++i) {
		this->led_string.channel[0].leds[i] = 0;
		ws2811_render(&(this->led_string));
	}
	ws2811_fini(&(this->led_string));
}

int main(int argc, char** argv)
{
	// Run ROS
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<LEDControl>());
	rclcpp::shutdown();
	return 0;
}
