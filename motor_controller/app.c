// Include libraries

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/header.h>
#include <std_msgs/msg/int32.h>

#include <stdio.h>
#include <unistd.h>
#include <time.h>

#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#include <driver/gpio.h>

#define STRING_BUFFER_LEN 50

#define LED_BUILTIN 2

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}


rcl_publisher_t dir_publisher;
rcl_subscription_t key_subscriber;

std_msgs__msg__Int32 key;
std_msgs__msg__Header dir;

void key_sub_callback(){

    if(key.data == 0){
        dir.frame_id.data = "Left arrow pressed";
        gpio_set_level(LED_BUILTIN, !gpio_get_level(LED_BUILTIN));
    }
    if(key.data == 1){
        dir.frame_id.data = "Up arrow pressed";
        gpio_set_level(LED_BUILTIN, !gpio_get_level(LED_BUILTIN));
    }
    if(key.data == 2){
        dir.frame_id.data = "Right arrow pressed";
        gpio_set_level(LED_BUILTIN, !gpio_get_level(LED_BUILTIN));
    }
    if(key.data == 3){
        dir.frame_id.data = "Down arrow pressed";
        gpio_set_level(LED_BUILTIN, !gpio_get_level(LED_BUILTIN));
    }

    rcl_publish(&dir_publisher, (const void*)&dir, NULL);
}


void appMain(void *argument){
    //setupPins();

    gpio_reset_pin(LED_BUILTIN);
    gpio_set_direction(LED_BUILTIN, GPIO_MODE_INPUT_OUTPUT);


    rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "motor_control", "", &support));

    RCCHECK(rclc_subscription_init_default(&key_subscriber, &node,
		    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/key"));

    RCCHECK(rclc_publisher_init_default(&dir_publisher, &node,
		    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header), "/dir"));

    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
	//RCCHECK(rclc_executor_add_timer(&executor, &timer));
	RCCHECK(rclc_executor_add_subscription(&executor, &key_subscriber, &key,
		&key_sub_callback, ON_NEW_DATA));
    
    
    
    while(1){
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
		usleep(10000);
	}

    RCCHECK(rcl_publisher_fini(&dir_publisher, &node));
	RCCHECK(rcl_subscription_fini(&key_subscriber, &node));
}
