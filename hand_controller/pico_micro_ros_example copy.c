#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float64.h>
#include <std_msgs/msg/u_int64.h>
#include <rmw_microros/rmw_microros.h>
#include <math.h>

#include "pico/stdlib.h"
#include "pico_uart_transports.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"



const uint LED_PIN = 25;
const uint PWM_PIN = 22;
const uint BUTTON_PIN = 21;

static double glob_vel_pos = 0.35 * 65025;
static double glob_vel_neg = 0.65 * 65025;
const double time_from_0_to_100_us = 7600000;
double curr_vel;

double current_pos = 100;
double target_position = 100;

double time_to_consume = 0;
absolute_time_t start_time;

int timer_delay_US = 1;
rcl_publisher_t publisher;
rcl_subscription_t subscriber;
std_msgs__msg__Float64 msg;
std_msgs__msg__Float64 input;



void do_homing(){
    double go = true;
    while(go){
        pwm_clear_irq(pwm_gpio_to_slice_num(PWM_PIN));
        if (gpio_get(BUTTON_PIN) == 0){
            go = false;
            pwm_set_gpio_level(PWM_PIN, 0.5 * 65025);
            break;
        }
        pwm_set_gpio_level(PWM_PIN, glob_vel_neg);
    }
    current_pos = 0;

}


void subscription_callback(const void * msgin){
    const std_msgs__msg__Float64 * i_msg = (const std_msgs__msg__Float64 *)msgin;
    target_position = i_msg->data;

    if(target_position != current_pos){
        time_to_consume = (target_position - current_pos) * time_from_0_to_100_us / 100;
    } else if (target_position == current_pos){
        time_to_consume = 0;
    }
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{   
    pwm_clear_irq(pwm_gpio_to_slice_num(PWM_PIN));

    // get sign of the velocity trough time_to_consume
    if (time_to_consume > 0){
        curr_vel = glob_vel_pos;
    } else if (time_to_consume < 0){
        curr_vel = glob_vel_neg;
    } else if (time_to_consume == 0){
        curr_vel = 0.5 * 65025;
    }

    start_time = get_absolute_time();
    while (absolute_time_diff_us(start_time, get_absolute_time()) <= abs(time_to_consume)) {
        pwm_set_gpio_level(PWM_PIN, curr_vel);
    }

    sleep_ms(3);

    msg.data = time_to_consume;
    rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);

    pwm_clear_irq(pwm_gpio_to_slice_num(PWM_PIN));
    pwm_set_gpio_level(PWM_PIN, 0.5 * 65025);
    current_pos = target_position;
    time_to_consume = 0;
}

int main()
{
    rmw_uros_set_custom_transport(
		true,
		NULL,
		pico_serial_transport_open,
		pico_serial_transport_close,
		pico_serial_transport_write,
		pico_serial_transport_read
	);

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_init(BUTTON_PIN);
    gpio_set_dir(BUTTON_PIN, GPIO_IN);
    gpio_pull_up(BUTTON_PIN); 
    

	

    rcl_timer_t timer;
    rcl_node_t node;
    rcl_allocator_t allocator;
    rclc_support_t support;
    rclc_executor_t executor;

    sleep_ms(3000);
   

    // init pwm
    gpio_set_function(PWM_PIN, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(PWM_PIN);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 5.7803);
    pwm_init(slice_num, &config, true);

    do_homing();

    allocator = rcl_get_default_allocator();

    // Wait for agent successful ping for 2 minutes.
    const int timeout_ms = 1000; 
    const uint8_t attempts = 120;

    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);

    if (ret != RCL_RET_OK)
    {
        // Unreachable agent, exiting program.
        return ret;
    }

    rclc_support_init(&support, 0, NULL, &allocator);

    rclc_node_init_default(&node, "hand_mcu_node", "", &support);
    rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
        "pico_publisher");

    rclc_subscription_init_default(
	        &subscriber,
	        &node,
	        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Float64),
	        "hand_joint_pos");


    rclc_timer_init_default(
        &timer,
        &support,
        RCL_US_TO_NS(timer_delay_US),
        timer_callback);

    gpio_put(LED_PIN, 1);

    rclc_executor_init(&executor, &support.context, 2, &allocator);
    rclc_executor_add_timer(&executor, &timer);

    rclc_executor_add_subscription(&executor,
			&subscriber,
			&input,
			&subscription_callback,
			ON_NEW_DATA);

    //msg.data = 0;
    while (true)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
    return 0;
}
