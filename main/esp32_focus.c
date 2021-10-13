#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "driver/mcpwm.h"
#include "driver/pcnt.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "driver/timer.h"
#include "hal/mcpwm_ll.h"

#define encoderPinA     34
#define encoderPinB     35

#define motorPinA       25
#define motorPinB       26

#define MAX_COUNTER     32767

#define TIMER_FREQ      1000000

volatile int32_t total_count = 0;
volatile uint32_t pulse_count;
volatile uint32_t last_value = 0;
volatile bool flag = false;
volatile bool direction;
volatile int64_t timer_value;

void overflowInterrupt(void *arg){
    uint32_t status;
    pcnt_get_event_status(PCNT_UNIT_0, &status);
    if(status & 32)        //overflow
        total_count += MAX_COUNTER;
    else if(status & 16)   //underflow
        total_count -= -(MAX_COUNTER+1);
}

uint64_t get_time(){ //get time in us
    timer_get_counter_value(TIMER_GROUP_0, TIMER_0, &timer_value);
    return 1000000*timer_value/TIMER_FREQ;
}

bool captureInterrupt(mcpwm_unit_t mcpwm, mcpwm_capture_channel_id_t cap_channel, const cap_event_data_t* edata, void* user_data){

    uint32_t capture = edata->cap_value;
    pulse_count = capture - last_value;
    last_value = capture;

    if(gpio_get_level(encoderPinB))
        direction = 1;
    else    
        direction = 0;

    //flag = true;
 
    return true;
}

void app_main(void)
{  
    mcpwm_config_t mcpwm_conf;
    mcpwm_capture_config_t mcpwm_capture_conf;
    pcnt_config_t  pcnt_conf;
    timer_config_t timer_conf;

    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, motorPinA);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, motorPinB);

    mcpwm_conf.frequency = 20000;
    mcpwm_conf.cmpr_a = 30;
    mcpwm_conf.cmpr_b = 0;
    mcpwm_conf.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_conf.counter_mode = MCPWM_UP_COUNTER;

    mcpwm_group_set_resolution(MCPWM_UNIT_0, 10000000);
    mcpwm_timer_set_resolution(MCPWM_UNIT_0, MCPWM_TIMER_0, 10000000);

    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &mcpwm_conf);

    //-------------------timer-----------------------//

    timer_conf.alarm_en = TIMER_ALARM_DIS;
    timer_conf.counter_en = TIMER_PAUSE;
    timer_conf.counter_dir = TIMER_COUNT_UP;
    timer_conf.divider = 80;

   
    timer_init(TIMER_GROUP_0, TIMER_0, &timer_conf);
  
    
    //-------------------encoder---------------------//

    pcnt_conf.unit = PCNT_UNIT_0;
    pcnt_conf.channel = PCNT_CHANNEL_0;

    pcnt_conf.pulse_gpio_num = encoderPinA;
    pcnt_conf.ctrl_gpio_num = encoderPinB;

    pcnt_conf.counter_l_lim = -(MAX_COUNTER+1);
    pcnt_conf.counter_h_lim = MAX_COUNTER;

    pcnt_conf.lctrl_mode = PCNT_CHANNEL_LEVEL_ACTION_KEEP; 
    pcnt_conf.hctrl_mode = PCNT_CHANNEL_LEVEL_ACTION_INVERSE; 

    pcnt_conf.pos_mode = PCNT_CHANNEL_EDGE_ACTION_INCREASE;
    pcnt_conf.neg_mode = PCNT_CHANNEL_EDGE_ACTION_DECREASE;

    pcnt_unit_config(&pcnt_conf);

    pcnt_conf.unit = PCNT_UNIT_0;
    pcnt_conf.channel = PCNT_CHANNEL_1;

    pcnt_conf.pulse_gpio_num = encoderPinB;
    pcnt_conf.ctrl_gpio_num = encoderPinA;

    pcnt_conf.counter_l_lim = -(MAX_COUNTER+1);
    pcnt_conf.counter_h_lim = MAX_COUNTER;

    pcnt_conf.lctrl_mode = PCNT_CHANNEL_LEVEL_ACTION_KEEP; 
    pcnt_conf.hctrl_mode = PCNT_CHANNEL_LEVEL_ACTION_INVERSE; 

    pcnt_conf.pos_mode = PCNT_CHANNEL_EDGE_ACTION_DECREASE;
    pcnt_conf.neg_mode = PCNT_CHANNEL_EDGE_ACTION_INCREASE;

    pcnt_unit_config(&pcnt_conf);

    pcnt_event_enable(PCNT_UNIT_0, PCNT_EVT_H_LIM);
    pcnt_event_enable(PCNT_UNIT_0, PCNT_EVT_L_LIM);

    pcnt_isr_service_install(0);
    pcnt_isr_handler_add(PCNT_UNIT_0, overflowInterrupt, NULL);

    pcnt_intr_enable(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_0);

    //------------------capture-------------//

    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_CAP_0, encoderPinA);

    mcpwm_capture_conf.cap_edge = MCPWM_BOTH_EDGE;
    mcpwm_capture_conf.cap_prescale = 6;
    mcpwm_capture_conf.capture_cb = captureInterrupt;
    mcpwm_capture_conf.user_data = NULL;

    mcpwm_capture_enable_channel(MCPWM_UNIT_0, MCPWM_SELECT_CAP0, &mcpwm_capture_conf);

    float duty[8] = {0, 35, 60, 100, 50, 80, 40, 0};
    uint64_t start_time;

    timer_start(TIMER_GROUP_0, TIMER_0);

    start_time = get_time();
    for(int i = 0; i < 8; i++){
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM0A, duty[i]);
        for(int j = 0; j<5000; j++){
            start_time += 1000;
            while(get_time() < start_time);
            printf("%llu %.2f %.2f %u %d\n", get_time(), duty[i], 0.0, pulse_count, direction);
        }
    }
}
