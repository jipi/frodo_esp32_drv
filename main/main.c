#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

//jipi
#include <geometry_msgs/msg/twist.h>


#include <driver/gpio.h>
#include <driver/ledc.h>

//jipi
#include <driver/adc.h>
#include "esp_adc_cal.h"

//#define LED_BUILTIN 33


#define PIN_RIGHT_FORWARD 13
#define PIN_LEFT_FORWARD 32

//right
#define PIN_IN3 23//17//14//17//2

#define PIN_IN4 5//27//5

//left
#define PIN_IN1 33

#define PIN_IN2 25//12



//ADC Channels
#if CONFIG_IDF_TARGET_ESP32
#define ADC1_EXAMPLE_CHAN0          ADC1_CHANNEL_0
#define ADC2_EXAMPLE_CHAN0          ADC2_CHANNEL_0
static const char *TAG_CH[2][10] = {{"ADC1_CH0"}, {"ADC2_CH0"}};
#else
#define ADC1_EXAMPLE_CHAN0          ADC1_CHANNEL_2
#define ADC2_EXAMPLE_CHAN0          ADC2_CHANNEL_0
static const char *TAG_CH[2][10] = {{"ADC1_CH2"}, {"ADC2_CH0"}};
#endif

//ADC Attenuation
#define ADC_EXAMPLE_ATTEN           ADC_ATTEN_DB_11

//ADC Calibration
#if CONFIG_IDF_TARGET_ESP32
#define ADC_EXAMPLE_CALI_SCHEME     ESP_ADC_CAL_VAL_EFUSE_VREF
#elif CONFIG_IDF_TARGET_ESP32S2
#define ADC_EXAMPLE_CALI_SCHEME     ESP_ADC_CAL_VAL_EFUSE_TP
#elif CONFIG_IDF_TARGET_ESP32C3
#define ADC_EXAMPLE_CALI_SCHEME     ESP_ADC_CAL_VAL_EFUSE_TP
#elif CONFIG_IDF_TARGET_ESP32S3
#define ADC_EXAMPLE_CALI_SCHEME     ESP_ADC_CAL_VAL_EFUSE_TP_FIT
#endif



// PWM Channels (Reserve channel 0 and 1 for camera)
#define PWM_LEFT_FORWARD LEDC_CHANNEL_2
//#define PWM_LEFT_BACKWARD LEDC_CHANNEL_3
#define PWM_RIGHT_FORWARD LEDC_CHANNEL_4
//#define PWM_RIGHT_BACKWARD LEDC_CHANNEL_5

// Other PWM settings
//#define PWM_FREQUENCY 1000 //50
// 1000 is for v2-00
// 250 is for v2-01
#define PWM_FREQUENCY 250 
#define PWM_RESOLUTION LEDC_TIMER_12_BIT
#define PWM_TIMER LEDC_TIMER_1
#define PWM_MODE LEDC_HIGH_SPEED_MODE

// These values are determined by experiment and are unique to every robot
//#define PWM_MOTOR_MIN 750    // The value where the motor starts moving
#define PWM_MOTOR_MIN 168
#define PWM_MOTOR_MAX 4092//4095   // Full speed (2^12 - 1)

// MACRO
#define constrain(amt, low, high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}


//ADC
//static int adc_raw[2][10];
static int adc_now;
//static int adc_cooked[1][1];
static const char *TAG = "ADC SINGLE";

static esp_adc_cal_characteristics_t adc1_chars;
static esp_adc_cal_characteristics_t adc2_chars;

static int i=0;
static unsigned int samples=0;
static int adc_avg=0;
static int adc_now=0;
static int adc_acc=0;


//MICROROS
rcl_publisher_t publisher;
rcl_subscription_t subscriber;
std_msgs__msg__Int32 send_msg;
std_msgs__msg__Int32 recv_msg;
geometry_msgs__msg__Twist cmd_vel;


//const int32_t ping_t = 1;//[millis] Pingtimeout


//helper

float fmap(float val, float in_min, float in_max, float out_min, float out_max) {
        return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


static bool adc_calibration_init(void)
{
    esp_err_t ret;
    bool cali_enable = false;

    ret = esp_adc_cal_check_efuse(ADC_EXAMPLE_CALI_SCHEME);
    if (ret == ESP_ERR_NOT_SUPPORTED) {
        ESP_LOGW(TAG, "Calibration scheme not supported, skip software calibration");
    } else if (ret == ESP_ERR_INVALID_VERSION) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else if (ret == ESP_OK) {
        cali_enable = true;
        esp_adc_cal_characterize(ADC_UNIT_1, ADC_EXAMPLE_ATTEN, ADC_WIDTH_BIT_12, 0, &adc1_chars);
        esp_adc_cal_characterize(ADC_UNIT_2, ADC_EXAMPLE_ATTEN, ADC_WIDTH_BIT_12, 0, &adc2_chars);
    } else {
        ESP_LOGE(TAG, "Invalid arg");
    }

    return cali_enable;
}


void setupPins() {

    // Led. Set it to GPIO_MODE_INPUT_OUTPUT, because we want to read back the state we set it to.
    //gpio_reset_pin(LED_BUILTIN);
    //gpio_set_direction(LED_BUILTIN, GPIO_MODE_INPUT_OUTPUT);

    // Configure timer
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = PWM_RESOLUTION,
        .freq_hz = PWM_FREQUENCY,
        .speed_mode = PWM_MODE,
        .timer_num = PWM_TIMER,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&ledc_timer);

    // Configure 4 PWM channels and assign output pins
    ledc_channel_config_t ledc_channel[2] = {
        {
            .channel    = PWM_LEFT_FORWARD,
            .duty       = 0,
            .gpio_num   = PIN_LEFT_FORWARD,
            .speed_mode = PWM_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_TIMER_1
        },
//        {
//            .channel    = PWM_LEFT_BACKWARD,
//            .duty       = 0,
//            .gpio_num   = PIN_LEFT_BACKWARD,
//            .speed_mode = PWM_MODE,
//            .hpoint     = 0,
//            .timer_sel  = LEDC_TIMER_1
//        },
        {
            .channel    = PWM_RIGHT_FORWARD,
            .duty       = 0,
            .gpio_num   = PIN_RIGHT_FORWARD,
            .speed_mode = PWM_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_TIMER_1
        },
//        {
//            .channel    = PWM_RIGHT_BACKWARD,
//            .duty       = 0,
//            .gpio_num   = PIN_RIGHT_BACKWARD,
//            .speed_mode = PWM_MODE,
//            .hpoint     = 0,
//            .timer_sel  = LEDC_TIMER_1
 //       },
    };

    for (int i = 0; i < 2; i++) {
        ledc_channel_config(&ledc_channel[i]);
    }

    //L798
    gpio_set_direction(PIN_IN1, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_IN2, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_IN3, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_IN4, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_IN1, 0);
    gpio_set_level(PIN_IN2, 0);
    gpio_set_level(PIN_IN3, 0);
    gpio_set_level(PIN_IN4, 0);


}


void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
    (void) last_call_time;

    // gpio_set_level(LED_BUILTIN, !gpio_get_level(LED_BUILTIN));

//	if (timer != NULL) {
		//RCSOFTCHECK(rcl_publish(&publisher, &send_msg, NULL));
		//printf("Sent: %d\n", send_msg.data);
		//send_msg.data++;
//	}

    // Use linear.x for forward value and angular.z for rotation
    float linear = constrain(cmd_vel.linear.x, -1, 1);
    float angular = constrain(cmd_vel.angular.z, -1, 1);

    // This robot is an RC tank and uses a differential drive (skid steering).
    // Calculate the speed of left and right motors. Simple version without wheel distances.
    // Taken from https://hackernoon.com/unicycle-to-differential-drive-courseras-control-of-mobile-robots-with-ros-and-rosbots-part-2-6d27d15f2010#1e59
    
//    if ((angular < 0.2f) || (angular > -0.2f)) {
      float left = (linear - angular) / 1.6f;
      float right = (linear + angular) / 1.6f;

      if ((angular > 0.2) || (angular < -0.2)) {
        left = (linear - angular) / 2.0f;
        right = (linear + angular) / 2.0f;
      }
    


    // Then map those values to PWM intensities. PWM_MOTOR_MAX = full speed, PWM_MOTOR_MIN = the minimal amount of power at which the motors begin moving.
    uint16_t pwmLeft = (uint16_t) fmap(fabs(left), 0, 1, PWM_MOTOR_MIN, PWM_MOTOR_MAX) *1.0;
    uint16_t pwmRight = (uint16_t) fmap(fabs(right), 0, 1, PWM_MOTOR_MIN, PWM_MOTOR_MAX)*1.0;

    // Each wheel has a channel for forwards and backwards movement
  //  ledc_set_duty(PWM_MODE, PWM_LEFT_FORWARD, pwmLeft * (left > 0));
//    ledc_set_duty(PWM_MODE, PWM_LEFT_BACKWARD, pwmLeft * (left < 0));
      ledc_set_duty(PWM_MODE, PWM_LEFT_FORWARD, pwmLeft * 1);
//      ledc_set_duty(PWM_MODE, PWM_LEFT_FORWARD, pwmLeft * left);

      ledc_set_duty(PWM_MODE, PWM_RIGHT_FORWARD, pwmRight * 1);
   // ledc_set_duty(PWM_MODE, PWM_RIGHT_FORWARD, pwmRight * (right > 0));
//    ledc_set_duty(PWM_MODE, PWM_RIGHT_BACKWARD, pwmRight * (right < 0));

      ledc_update_duty(PWM_MODE, PWM_LEFT_FORWARD);
  //  ledc_update_duty(PWM_MODE, PWM_LEFT_BACKWARD);
      ledc_update_duty(PWM_MODE, PWM_RIGHT_FORWARD);
    //ledc_update_duty(PWM_MODE, PWM_RIGHT_BACKWARD);

     if (left>0 && right>0) {
        gpio_set_level(PIN_IN1, 1);
        gpio_set_level(PIN_IN2, 0);
        gpio_set_level(PIN_IN3, 1);
        gpio_set_level(PIN_IN4, 0);
     } else if (left>0 && right<0) {
        gpio_set_level(PIN_IN1, 1);
        gpio_set_level(PIN_IN2, 0);
        gpio_set_level(PIN_IN3, 0);
        gpio_set_level(PIN_IN4, 1);
     } else if (left<0 && right>0) {
        gpio_set_level(PIN_IN1, 0);
        gpio_set_level(PIN_IN2, 1);
        gpio_set_level(PIN_IN3, 1);
        gpio_set_level(PIN_IN4, 0);
     } else if (left<0 && right<0) {
        gpio_set_level(PIN_IN1, 0);
        gpio_set_level(PIN_IN2, 1);
        gpio_set_level(PIN_IN3, 0);
        gpio_set_level(PIN_IN4, 1);
     } else {
        gpio_set_level(PIN_IN1, 1);
        gpio_set_level(PIN_IN2, 1);
        gpio_set_level(PIN_IN3, 1);
        gpio_set_level(PIN_IN4, 1);
     }



    if (cmd_vel.linear.x == 0 && cmd_vel.angular.z ==0){
        gpio_set_level(PIN_IN1, 0);
        gpio_set_level(PIN_IN2, 0);
        gpio_set_level(PIN_IN3, 0);
        gpio_set_level(PIN_IN4, 0);
        ledc_set_duty(PWM_MODE, PWM_LEFT_FORWARD, 0);
        ledc_set_duty(PWM_MODE, PWM_RIGHT_FORWARD, 0);
    }



#if 0
    if (cmd_vel.linear.x > 0.0) {
        gpio_set_level(PIN_IN1, 1);
        gpio_set_level(PIN_IN2, 0);
        gpio_set_level(PIN_IN3, 1);
        gpio_set_level(PIN_IN4, 0);
        printf("gpio set +\n");

    } else if (cmd_vel.linear.x < 0.0) {
        gpio_set_level(PIN_IN1, 0);
        gpio_set_level(PIN_IN2, 1);
        gpio_set_level(PIN_IN3, 0);
        gpio_set_level(PIN_IN4, 1);
        printf("gpio set -\n");
   
    } else if (cmd_vel.linear.x == 0 ){
        gpio_set_level(PIN_IN1, 0);
        gpio_set_level(PIN_IN2, 0);
        gpio_set_level(PIN_IN3, 0);
        gpio_set_level(PIN_IN4, 0);
        printf("gpio set 0\n");
    }

    if (cmd_vel.angular.z > 0.0) {
        //gpio_set_level(PIN_IN1, 1);
        //gpio_set_level(PIN_IN2, 0);
        gpio_set_level(PIN_IN3, 1);
        gpio_set_level(PIN_IN4, 0);
        printf("turn left\n");

    } else if (cmd_vel.angular.z < 0.0) {
        gpio_set_level(PIN_IN1, 1);
        gpio_set_level(PIN_IN2, 0);
        //gpio_set_level(PIN_IN3, 0);
        //gpio_set_level(PIN_IN4, 1);
        printf("turn right\n");
    } 

#endif




    printf("%d, %d %d, %d, %d %d, %f, %f\n", pwmLeft, left > 0, left < 0, pwmRight, right > 0, right < 0, left, right);
    printf("cmdvellinx%f, lin%f, cmdvelangz%f, ang%f\n", cmd_vel.linear.x, linear, cmd_vel.angular.z, angular);

  //temporary adc code

    ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_BIT_12));
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC1_EXAMPLE_CHAN0, ADC_EXAMPLE_ATTEN));
    adc_now = adc1_get_raw(ADC1_EXAMPLE_CHAN0);
    adc_acc += adc_now;
    samples++;
    
    if (i%16==1) {
        //uint32_t voltage = 0;

        //ADC1 config
        //ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_BIT_12));
        //ESP_ERROR_CHECK(adc1_config_channel_atten(ADC1_EXAMPLE_CHAN0, ADC_EXAMPLE_ATTEN));

        //adc_raw[0][0] = adc1_get_raw(ADC1_EXAMPLE_CHAN0);
        //ESP_LOGI(TAG_CH[0][0], "raw  data: %d", adc_raw[0][0]);
        //voltage = esp_adc_cal_raw_to_voltage(adc_raw[0][0], &adc1_chars);
        //adc_cooked[0][0] = voltage;
        //ESP_LOGI(TAG_CH[0][0], "cali data: %d mV", voltage);
    
        adc_avg = adc_acc/16;
        adc_acc = 0;
        samples = 0;
        ESP_LOGI(TAG_CH[0][0], "avg data: %d mV", adc_avg);


        //i++;
    }
    i++;

    if (timer != NULL) {
         //adc_cooked[0][0] = voltage;
         RCSOFTCHECK(rcl_publish(&publisher, &adc_avg, NULL));
        //printf("Sent: %d\n", .data);
        //send_msg.data++;
    }

}

void subscription_callback(const void * msgin)
{
	//const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
    geometry_msgs__msg__Twist * msg = (geometry_msgs__msg__Twist *) msgin;
	printf("Received: %f %f\n", msg->linear.x, msg->angular.z);
}

void micro_ros_task(void * arg)
{
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;


      // jipi
    //usleep(10000000);
    setupPins();

    // ADC setup
    esp_err_t ret = ESP_OK;
    bool cali_enable = adc_calibration_init();

    //ADC1 config
    ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_BIT_12));
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC1_EXAMPLE_CHAN0, ADC_EXAMPLE_ATTEN));




	// Create init_options.
	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	RCCHECK(rcl_init_options_init(&init_options, allocator));

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
	rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

	// Static Agent IP and port can be used instead of autodisvery.
	RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
	//RCCHECK(rmw_uros_discover_agent(rmw_options));
#endif
	// Setup support structure.
	RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

	// Create node.
	rcl_node_t node = rcl_get_zero_initialized_node();
	RCCHECK(rclc_node_init_default(&node, "ros_esp32_drv", "", &support));

	// Create publisher.
	RCCHECK(rclc_publisher_init_best_effort(
		&publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"int32_publisher"));

	// Create subscriber.
	RCCHECK(rclc_subscription_init_default(
		&subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
		// jipi"cmd_vel"));
        "velocity_smoother/smoothed"));

	// Create timer.
	rcl_timer_t timer = rcl_get_zero_initialized_timer();
	// jipiconst unsigned int timer_timeout = 1000;
    const unsigned int timer_timeout = 200;
	RCCHECK(rclc_timer_init_default(
		&timer,
		&support,
		RCL_MS_TO_NS(timer_timeout),
		timer_callback));

	// Create executor.
	rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
	RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
	// jipiunsigned int rcl_wait_timeout = 1000;   // in ms
    unsigned int rcl_wait_timeout = 1000;   // in ms


	RCCHECK(rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(rcl_wait_timeout)));

	// Add timer and subscriber to executor.
	RCCHECK(rclc_executor_add_timer(&executor, &timer));
	//RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &recv_msg, &subscription_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &cmd_vel, &subscription_callback, ON_NEW_DATA));


#if 0
    // jipi
    setupPins();

    // ADC setup
    esp_err_t ret = ESP_OK;
    bool cali_enable = adc_calibration_init();

    //ADC1 config
    ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_BIT_12));
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC1_EXAMPLE_CHAN0, ADC_EXAMPLE_ATTEN));
#endif

    // Spin forever.
	send_msg.data = 0;
	while(1){

        //if (RMW_RET_OK == rmw_uros_ping_agent(ping_t, 10)){
		    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

		//jipiusleep(100000);
            usleep(20000);
        //}
	}

	// Free resources.
	RCCHECK(rcl_subscription_fini(&subscriber, &node));
	RCCHECK(rcl_publisher_fini(&publisher, &node));
	RCCHECK(rcl_node_fini(&node));

  	vTaskDelete(NULL);
}

void app_main(void)
{

//   usleep(8000000);


#ifdef CONFIG_MICRO_ROS_ESP_NETIF_WLAN || CONFIG_MICRO_ROS_ESP_NETIF_ENET
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif

    //usleep(8000000);


    //pin micro-ros task in APP_CPU to make PRO_CPU to deal with wifi:
    xTaskCreate(micro_ros_task,
            "uros_task",
            CONFIG_MICRO_ROS_APP_STACK,
            NULL,
            CONFIG_MICRO_ROS_APP_TASK_PRIO,
            NULL);
}
