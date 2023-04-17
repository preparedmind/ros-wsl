//ros libraries
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
//ros message libraries
#include <std_msgs/msg/header.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float64.h>
//c libraries
#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
//freertos libraries
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
//esp32 driver libraries
#include <driver/gpio.h>
#include <driver/ledc.h>
//define pins
#define LED_BUILTIN 2
#define Mforward 21
#define Mbackward 19
#define Ena 5
#define Enb 18
#define PWM_LEFT_FORWARD LEDC_CHANNEL_2
#define PWM_LEFT_BACKWARD LEDC_CHANNEL_3
//pwm settings
#define PWM_FREQUENCY 50
#define PWM_RESOLUTION LEDC_TIMER_12_BIT
#define PWM_TIMER LEDC_TIMER_1
#define PWM_MODE LEDC_HIGH_SPEED_MODE
//interrupt pin
#define ESP_INTR_FLAG_DEFAULT 0
//max char bits
#define MAX 100
//define ros setup functions
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}
//create pubs and subs
rcl_subscription_t key_subscriber;
rcl_publisher_t ena_publisher;
rcl_publisher_t enb_publisher;
//create messages
std_msgs__msg__Int32 key;
std_msgs__msg__Int32 ena;
std_msgs__msg__Header enb;
//encoder pulses since last reset, distance in encoder pulses from starting point, return val in dFromV
int currentECount, position, d= 0;
//times for averaging velocity and times for when to publish velocity
clock_t old_t, new_t, end_t, start_t;
//turn double velocity into char for publishing
char buf[MAX];
//Estimated pulses per revolution
int ppr = 1080;
//old and new number of encoders travelled, old and new time since pulses last measured, velocity when last published, time difference
double oldp, newp, oldt, newt, oldvel, diff = 0;
//y intercept for dFromV()
double b = ((500.0/60.0)*510.0)-3595.0;
//interrupt management
static QueueHandle_t gpio_evt_queue = NULL;
//PWM setup
ledc_channel_config_t ledc_channel[2];
//get duty cycle from velocity(deg/s)
int dFromV(double v){
    if (v>=0){
        d = ((500/60)*v)-b;
    }
    else if(v<0){
        d = -1*(((500/60)*fabs(v))-b);
    }
    return d;
}
//total degrees travelled
double degtrav(int p){
    return (double)p/3.0;
}
//average of velocity over the last 1/2 second
double vel(){
    new_t = clock();
    newp = degtrav(currentECount);
    diff = difftime(new_t, old_t);
    diff = diff/1000.0;
    if(diff >= 0.5){
        currentECount = 0;
        old_t = clock();
        new_t = clock();
        diff = difftime(new_t, old_t);
        diff = diff/1000.0;
        oldp = degtrav(currentECount);
        newp = degtrav(currentECount);
    }
    double dp = newp-oldp;
    return dp/diff; 
}
//set motors to turn
void drive(int vel){
    if (vel > 0){
        ledc_set_duty(PWM_MODE, PWM_LEFT_FORWARD, vel);
        ledc_set_duty(PWM_MODE, PWM_LEFT_BACKWARD, 0);
        ledc_update_duty(PWM_MODE, PWM_LEFT_FORWARD);
        ledc_update_duty(PWM_MODE, PWM_LEFT_BACKWARD);
    }
    else if (vel < 0){
        ledc_set_duty(PWM_MODE, PWM_LEFT_FORWARD, 0);
        ledc_set_duty(PWM_MODE, PWM_LEFT_BACKWARD, abs(vel));
        ledc_update_duty(PWM_MODE, PWM_LEFT_FORWARD);
        ledc_update_duty(PWM_MODE, PWM_LEFT_BACKWARD);
    }
    old_t = clock();
    oldp = degtrav(currentECount);
}
//When recieving key press turn motor at speed depending on which key pressed
void key_sub_callback(){
    if(key.data == 0){
       //left arrow
        gpio_set_level(LED_BUILTIN, !gpio_get_level(LED_BUILTIN));
        drive(dFromV(360));
    }
    if(key.data == 1){
        //up arrow
        gpio_set_level(LED_BUILTIN, !gpio_get_level(LED_BUILTIN));
        drive(dFromV(180)); 
    }
    if(key.data == 2){
        //right arrow
        gpio_set_level(LED_BUILTIN, !gpio_get_level(LED_BUILTIN));
        drive(dFromV(-360));
    }
    if(key.data == 3){
        //down arrow
        gpio_set_level(LED_BUILTIN, !gpio_get_level(LED_BUILTIN));
        drive(dFromV(-180));        
    }
}
// change the position when encoder changes
static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    if(gpio_get_level(Enb)>0){
       position++;
       //currentECount++;
    }
    else{
        position--;
        //currentECount--;
    }

    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}
void appMain(void *argument){
    //start time
    start_t = clock();
    //led pin setup
    gpio_reset_pin(LED_BUILTIN);
    gpio_set_direction(LED_BUILTIN, GPIO_MODE_INPUT_OUTPUT);
    //motor pins
    gpio_reset_pin(Mforward);
    gpio_set_direction(Mforward, GPIO_MODE_INPUT_OUTPUT);
    gpio_reset_pin(Mbackward);
    gpio_set_direction(Mbackward, GPIO_MODE_INPUT_OUTPUT);
    //encoder pins
    gpio_reset_pin(Ena);
    gpio_set_direction(Ena, GPIO_MODE_INPUT);
    gpio_reset_pin(Enb);
    gpio_set_direction(Enb, GPIO_MODE_INPUT);
    //interrupt for encoder change
    gpio_set_intr_type(Ena, GPIO_INTR_POSEDGE);
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(Ena, gpio_isr_handler, (void*) Ena);
    //interrupt management
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    //pwm setup
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = PWM_RESOLUTION,
        .freq_hz = PWM_FREQUENCY,
        .speed_mode = PWM_MODE,
        .timer_num = PWM_TIMER,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&ledc_timer);
    ledc_channel_config_t ledc_channel[2] = {
        {
            .channel    = PWM_LEFT_FORWARD,
            .duty       = 0,
            .gpio_num   = Mforward,
            .speed_mode = PWM_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_TIMER_1
        },
        {
            .channel    = PWM_LEFT_BACKWARD,
            .duty       = 0,
            .gpio_num   = Mbackward,
            .speed_mode = PWM_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_TIMER_1
        }
    };
    for (int i = 0; i < 2; i++) {
        ledc_channel_config(&ledc_channel[i]);
    }
    //ros setup
    rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "motor_control", "", &support));
    //sub and pub setup
    RCCHECK(rclc_subscription_init_default(&key_subscriber, &node,
		    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/key"));
    RCCHECK(rclc_publisher_init_default(&ena_publisher, &node,
	    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/ena"));
    RCCHECK(rclc_publisher_init_default(&enb_publisher, &node,
	    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header), "/enb"));
    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
	RCCHECK(rclc_executor_add_subscription(&executor, &key_subscriber, &key,
		&key_sub_callback, ON_NEW_DATA));
    //main loop
    while(1){
        //spin ros node
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
		//usleep(10000);
        //publish velocity every 1/2 second
        end_t = clock();
        double gap = difftime(end_t, start_t);
        if (gap >= 0.5){
            //if the current velocity is real and has changed since last measured publish
            double curvel = vel();
            if(!(isnan(curvel)) && fabs(oldvel-curvel) >= 5.0 && isfinite(curvel)){
                sprintf(buf, "%f", curvel);
                enb.frame_id.data = buf;            
                rcl_publish(&enb_publisher, (const void*)&enb, NULL);
                oldvel = vel();
            }
        }
        //publish position
        ena.data = position;
        rcl_publish(&ena_publisher, (const void*)&ena, NULL);
	}
    //close pubs and sub
    RCCHECK(rcl_publisher_fini(&ena_publisher, &node));
    RCCHECK(rcl_publisher_fini(&enb_publisher, &node));
	RCCHECK(rcl_subscription_fini(&key_subscriber, &node));
}