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



const uint LED_PIN = 25;
const uint PWR_PIN = 20;
const uint DIR_PINS[] = {7,11,1,4};
const uint STEP_PINS[] = {8,10,2,5}; //axis 0 was 8
const uint EN_PINS[] = {6,9,0,3};
const uint BUTTON_PINS[] = {19,18,17,16};



rcl_publisher_t publisher;
rcl_subscription_t subscriber;
std_msgs__msg__Float64 msg;
std_msgs__msg__UInt64 input;



const double pi = 3.14159265358979311599796346854;


const int micro_stepping_multiplier = 1;
const int steps_per_rev = 800*micro_stepping_multiplier;
const int gear_ratio_small = 38;
const int gear_ratio_big = 50;
const int steps_per_rev_w_gears[4] = {steps_per_rev*gear_ratio_big,steps_per_rev*gear_ratio_big,steps_per_rev*gear_ratio_small,steps_per_rev*gear_ratio_small};
const double rad_per_step[4] = {(2*pi)/steps_per_rev_w_gears[0], (2*pi)/steps_per_rev_w_gears[1], (2*pi)/steps_per_rev_w_gears[2], (2*pi)/steps_per_rev_w_gears[3]};
const double step_per_rad[4] = {steps_per_rev_w_gears[0]/(2*pi), steps_per_rev_w_gears[1]/(2*pi), steps_per_rev_w_gears[2]/(2*pi), steps_per_rev_w_gears[3]/(2*pi)};

uint64_t position;
double next_position[4];
int32_t signs_block;


double current_position[4];
bool dir[4];
bool signs[4];
double rad_to_do[4];
int steps_to_do[4] = {0,0,0,0};
int max_speed = 10;
const int homing_directions[4] = {0,0,0,0};
const double home_position[4] = {0,0.785, -1.5708, 0.0};
int homing_speed_delay_us = 300;

int steps_to_do_save[4];
int g = 0;

int timer_delay_US = 1;
int step_delay = 850;
int test = 0;
int counter[4] = {200,200,200,200};


void do_homing(int i){

    int trigger_count = 0;

    for(int b = 0; b < 4; b++){
        if(b != i){
            gpio_put(EN_PINS[b], 1);
        } else {
            gpio_put(EN_PINS[b], 0 );
        }
    }

    gpio_put(DIR_PINS[i], homing_directions[i]);

    sleep_ms(100);

    while (trigger_count < 2) {
        if (gpio_get(BUTTON_PINS[i]) == 0) {

            gpio_put(STEP_PINS[i], 1);
            sleep_us(homing_speed_delay_us);
            gpio_put(STEP_PINS[i], 0);  
            sleep_us(homing_speed_delay_us);
            trigger_count = 0;
        } else {
            // If the switch is triggered
            trigger_count++;
        }
    }

    next_position[i] = home_position[i];
    current_position[i] = home_position[i];
   
}


void subscription_callback(const void * msgin){
    const std_msgs__msg__UInt64 * i_msg = (const std_msgs__msg__UInt64 *)msgin;
    
    position = i_msg->data;
    signs_block = (uint64_t)(position*pow(10,-16));

    
    next_position[0] = (position - (uint64_t)(position*pow(10,-4))*(uint64_t)pow(10,4))*pow(10,-3);
    next_position[1] = (position - (uint64_t)(position*pow(10,-8))*(uint64_t)pow(10,8))*pow(10,-7);
    next_position[2] = (position - (uint64_t)(position*pow(10,-12))*(uint64_t)pow(10,12))*pow(10,-11);
    next_position[3] = (position - (uint64_t)(position*pow(10,-16))*(uint64_t)pow(10,16))*pow(10,-15);
    
    signs[0] = (bool)((uint64_t)(signs_block - ((uint64_t)(signs_block*pow(10,-1)))*(uint64_t)pow(10,1)));
    signs[1] = (bool)((uint64_t)((uint64_t)(signs_block - ((uint64_t)(signs_block*pow(10,-2)))*(uint64_t)pow(10,2))*pow(10,-1)));
    signs[2] = (bool)((uint64_t)((uint64_t)(signs_block - ((uint64_t)(signs_block*pow(10,-3)))*(uint64_t)pow(10,3))*pow(10,-2)));
    signs[3] = (bool)((uint64_t)(signs_block*pow(10,-3)));

    for(int i = 0; i < 4; i++){

        if(signs[i]){
            next_position[i] = -next_position[i];
        }
        rad_to_do[i] = current_position[i] - next_position[i];
        steps_to_do[i] = (int)(rad_to_do[i]*step_per_rad[i]);
        steps_to_do_save[i] = steps_to_do[i];

        if(current_position[i] > next_position[i]){
            dir[i] = 1;
        } else {
            dir[i] = 0;
            steps_to_do[i] = -steps_to_do[i];
        }

    }
    

}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{   
    
    test++;

    for(int i = 0; i < 4; i++){
        if ((steps_to_do[i] == 0) && (counter[i] > 0)){
            counter[i] -= 1;
        } else if (steps_to_do[i] != 0) {
            counter[i] = 100;
        }

        if (counter[i] == 0){
            if (i==2){
                gpio_put(EN_PINS[i], 0);
            } else {
                gpio_put(EN_PINS[i], 1);
            }
        } else {
            gpio_put(EN_PINS[i], 0);
        }

        if(steps_to_do[i] > 0){
           if (i==3 || i==2) {
                gpio_put(DIR_PINS[i], dir[i]);
            } else{
                gpio_put(DIR_PINS[i], !dir[i]);
            }
            gpio_put(STEP_PINS[i], 1);
            if(dir[i]){
                current_position[i] -= rad_per_step[i];
            } else {
                current_position[i] += rad_per_step[i];
            }
            steps_to_do[i]--;
        } 
    }

    sleep_us(step_delay);

    for(int i = 0; i < 4; i++){
        gpio_put(STEP_PINS[i], 0);
    }

    sleep_us(4);
    

    // msg.data = counter[g];
    // if(g == 3){
    //     g = 0;
    // } else {
    //     g++;
    // }
    // rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);
    
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
    for(int i = 0; i < 4; i++){
        gpio_init(STEP_PINS[i]);
        gpio_set_dir(STEP_PINS[i], GPIO_OUT);
        gpio_init(DIR_PINS[i]);
        gpio_set_dir(DIR_PINS[i], GPIO_OUT);
        gpio_init(EN_PINS[i]);
        gpio_set_dir(EN_PINS[i], GPIO_OUT);
        gpio_put(EN_PINS[i], 0);
        gpio_init(BUTTON_PINS[i]);
        gpio_set_dir(BUTTON_PINS[i], GPIO_IN);
        gpio_init(PWR_PIN);
        gpio_set_dir(PWR_PIN, GPIO_IN);

    }

	

    rcl_timer_t timer;
    rcl_node_t node;
    rcl_allocator_t allocator;
    rclc_support_t support;
    rclc_executor_t executor;

    int trigger = 0;

    while(trigger < 3){
        if(gpio_get(PWR_PIN)){
            trigger++;
        } else {
            trigger = 0;
        }
    }

    sleep_ms(3000);

    do_homing(1);
    do_homing(2);
    do_homing(3);
    do_homing(0);

   

    for(int b = 0; b < 4; b++){

        gpio_put(EN_PINS[b], 0);
        
    }


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

    rclc_node_init_default(&node, "arm_mcu_node", "", &support);
    // rclc_publisher_init_default(
    //     &publisher,
    //     &node,
    //     ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
    //     "pico_publisher");

    rclc_subscription_init_default(
	        &subscriber,
	        &node,
	        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,UInt64),
	        "arm_joint_pos");


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
