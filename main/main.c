#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/timers.h>

#include <ssd1306.h>
#include <string.h>
#include "led_strip.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "presence_bar.h"
#include "esp_sleep.h"
#include <esp_task_wdt.h> 
#include "wifiInit.h"
#include "timeMgmt.h"
#include "mqtt.h"
#include "esp_sntp.h"

#include <stdio.h>

TimerHandle_t deepSleepTimer;
QueueHandle_t detections;
SemaphoreHandle_t publishDetectionsSemaphore = NULL;

/**
   This interrupt handler is called whenever the outer photoelectric barrier is broken.
*/
void IRAM_ATTR pirInterrupt(void* arg){
	ets_printf("PIR wakeup\n");
}

#define LED_STRIP_LENGTH 103U
static struct led_color_t led_strip_buf_1[LED_STRIP_LENGTH];
static struct led_color_t led_strip_buf_2[LED_STRIP_LENGTH];

     struct led_strip_t led_strip = {
        .rgb_led_type = RGB_LED_TYPE_WS2812,
        .rmt_channel = RMT_CHANNEL_1,
        .rmt_interrupt_num = 32,
        .gpio = 32,
        .led_strip_buf_1 = led_strip_buf_1,
        .led_strip_buf_2 = led_strip_buf_2,
        .led_strip_length = LED_STRIP_LENGTH
    };



const uint8_t lights[360]={
  0,   0,   0,   0,   0,   1,   1,   2, 
  2,   3,   4,   5,   6,   7,   8,   9, 
 11,  12,  13,  15,  17,  18,  20,  22, 
 24,  26,  28,  30,  32,  35,  37,  39, 
 42,  44,  47,  49,  52,  55,  58,  60, 
 63,  66,  69,  72,  75,  78,  81,  85, 
 88,  91,  94,  97, 101, 104, 107, 111, 
114, 117, 121, 124, 127, 131, 134, 137, 
141, 144, 147, 150, 154, 157, 160, 163, 
167, 170, 173, 176, 179, 182, 185, 188, 
191, 194, 197, 200, 202, 205, 208, 210, 
213, 215, 217, 220, 222, 224, 226, 229, 
231, 232, 234, 236, 238, 239, 241, 242, 
244, 245, 246, 248, 249, 250, 251, 251, 
252, 253, 253, 254, 254, 255, 255, 255, 
255, 255, 255, 255, 254, 254, 253, 253, 
252, 251, 251, 250, 249, 248, 246, 245, 
244, 242, 241, 239, 238, 236, 234, 232, 
231, 229, 226, 224, 222, 220, 217, 215, 
213, 210, 208, 205, 202, 200, 197, 194, 
191, 188, 185, 182, 179, 176, 173, 170, 
167, 163, 160, 157, 154, 150, 147, 144, 
141, 137, 134, 131, 127, 124, 121, 117, 
114, 111, 107, 104, 101,  97,  94,  91, 
 88,  85,  81,  78,  75,  72,  69,  66, 
 63,  60,  58,  55,  52,  49,  47,  44, 
 42,  39,  37,  35,  32,  30,  28,  26, 
 24,  22,  20,  18,  17,  15,  13,  12, 
 11,   9,   8,   7,   6,   5,   4,   3, 
  2,   2,   1,   1,   0,   0,   0,   0, 
  0,   0,   0,   0,   0,   0,   0,   0, 
  0,   0,   0,   0,   0,   0,   0,   0, 
  0,   0,   0,   0,   0,   0,   0,   0, 
  0,   0,   0,   0,   0,   0,   0,   0, 
  0,   0,   0,   0,   0,   0,   0,   0, 
  0,   0,   0,   0,   0,   0,   0,   0, 
  0,   0,   0,   0,   0,   0,   0,   0, 
  0,   0,   0,   0,   0,   0,   0,   0, 
  0,   0,   0,   0,   0,   0,   0,   0, 
  0,   0,   0,   0,   0,   0,   0,   0, 
  0,   0,   0,   0,   0,   0,   0,   0, 
  0,   0,   0,   0,   0,   0,   0,   0, 
  0,   0,   0,   0,   0,   0,   0,   0, 
  0,   0,   0,   0,   0,   0,   0,   0, 
  0,   0,   0,   0,   0,   0,   0,   0};





void ultrasonicInit(void){
}

void distanceMeasurement(void){
 
        //ESP_ERROR_CHECK(ina3221_get_bus_voltage(&dev, i, &bus_voltage));
         ssd1306_clearScreen();
         char buf[200];
        sprintf(buf, "%.02f", 1.2);
        ssd1306_printFixedN(0, 0, buf, STYLE_NORMAL, 1);
        // sprintf(buf, "%.02f mV", shunt_voltage);
        // ssd1306_printFixedN(0, 20, buf, STYLE_NORMAL, 1);
        // sprintf(buf, "%.02f mA", shunt_current);
        // ssd1306_printFixedN(0, 40, buf, STYLE_NORMAL, 1);

}

void measureTask(void *pvParameters)
{

    while (1)
    {
        distanceMeasurement();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}



#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <ssd1306.h>
#include <esp_timer.h>
#include <string.h>
#include "driver/gpio.h"
#include <esp_log.h>



#define TIMEOUT_EXPIRED(start, len) ((esp_timer_get_time() - (start)) >= (len))


uint64_t my_micros(){
    uint64_t cycles=xthal_get_ccount();
    uint64_t m=(cycles/ets_get_cpu_frequency());
    return(m);
}

uint64_t micros_diff(uint64_t begin, uint64_t end) {
    return end - begin;
}

void delay_microseconds(int delay){
    uint64_t start = esp_timer_get_time();
    uint64_t end = start + delay;

    while (esp_timer_get_time()< end){}
}

uint64_t get_echo(uint32_t pin, uint32_t timeout) {
    uint64_t begin = esp_timer_get_time();

    // wait for any previous pulse to end
    while (gpio_get_level(pin)){
        if (TIMEOUT_EXPIRED(begin, timeout)) {
            ESP_LOGI("DISTANCE","Continuously high\n");
            return 1;
        }
    }

    // wait for the pulse to start
    while (!gpio_get_level(pin)){
        if (TIMEOUT_EXPIRED(begin, timeout)) {
            ESP_LOGI("DISTANCE","Echo did not start\n");
            return 2;
        }
    } 

    uint64_t pulseBegin = esp_timer_get_time();

    // wait for the pulse to stop
    while (gpio_get_level(pin)){
         if (TIMEOUT_EXPIRED(begin, timeout)) {
            ESP_LOGI("DISTANCE","Echo did not end\n");
            return 3;
        }
    }
    uint64_t pulseEnd = esp_timer_get_time();
    //ESP_LOGI("DISTANCE","micros diff = %lld", micros_diff(pulseBegin, pulseEnd) );

    return micros_diff(pulseBegin, pulseEnd);
}

long minDist;
long maxDist;

#define REPETITIONS 1
int measurementDelay=9000;
int lowThreshold=20, highThreshold=135;


/*The measured distance from the range 0 to 400 Centimeters*/
long measureInCentimeters(uint32_t pin) {
    minDist = 600;
    maxDist = 0;
    long averDist = 0;
  
    for (int i=0;i<REPETITIONS;i++){
        ESP_ERROR_CHECK(gpio_set_direction(pin, GPIO_MODE_OUTPUT));
        ESP_ERROR_CHECK(gpio_pulldown_en(pin));   
        ESP_ERROR_CHECK(gpio_set_level(pin, LOW));
        delay_microseconds(5);
        ESP_ERROR_CHECK(gpio_set_level(pin, 1));
        delay_microseconds(10);
        ESP_ERROR_CHECK(gpio_set_level(pin, LOW));
        ESP_ERROR_CHECK(gpio_set_direction(pin, GPIO_MODE_INPUT));
        ESP_ERROR_CHECK(gpio_pulldown_en(pin));
        uint64_t duration;
        duration = get_echo(pin, 100000L);
        long rangeInCentimeters;
        rangeInCentimeters = duration / 29 / 2;
        if (rangeInCentimeters>lowThreshold && rangeInCentimeters<highThreshold){
            if (minDist>rangeInCentimeters) minDist=rangeInCentimeters;
            if (maxDist<rangeInCentimeters) maxDist=rangeInCentimeters;
            averDist+=rangeInCentimeters;
        }
        delay_microseconds(500);
    }
    //ESP_LOGI("DISTANCE","min: %ld  max: %ld avg: %ld", minDist, maxDist, averDist/REPETITIONS ); 

    return averDist/REPETITIONS;
}


int distanceMeasurementCM(void){
    long distance[8];
    int cm;
    int minIndex=-1;
    static int call=0;

    //if (call>10) return(-1);
    //call++;

    
    distance[0]=0;
    distance[1]=0;
    distance[2]=0;
    distance[3]=0;
    distance[4]=0;
    distance[5]=0;
    distance[6]=0;
    distance[7]=0;

    distance[0]=measureInCentimeters(U1_GPIO); delay_microseconds(measurementDelay);
    distance[1]=measureInCentimeters(U2_GPIO); delay_microseconds(measurementDelay);
    distance[2]=measureInCentimeters(U3_GPIO); delay_microseconds(measurementDelay);
    distance[3]=measureInCentimeters(U4_GPIO); delay_microseconds(measurementDelay);
    distance[4]=measureInCentimeters(U5_GPIO); delay_microseconds(measurementDelay);
    distance[5]=measureInCentimeters(U6_GPIO); delay_microseconds(measurementDelay);
    distance[6]=measureInCentimeters(U7_GPIO); delay_microseconds(measurementDelay);
    distance[7]=measureInCentimeters(U8_GPIO); delay_microseconds(measurementDelay);
    //delay_microseconds(10000000);
 
    //ESP_LOGI("DISTANCE", "\n\n");
    for (int i=0;i<8;i++){
        if (distance[i]!=0) {
            ESP_LOGI("DISTANCE", "Sensor %d: %ld", i, distance[i]);
            if (minIndex== -1) 
                minIndex=i;
            else
                if (distance[i]<distance[minIndex]) minIndex=i;
        }
    }
    if (minIndex>-1) ESP_LOGI("DISTANCE", "minIndex: %d",minIndex);

    if (minIndex== -1 || distance[minIndex]>highThreshold){
        cm = -1;
    } else {
        cm=47*(minIndex+1)-23+8;
    }
    //return(-1);
    return(cm);
}


void eraseLEDStrip(){
    led_strip_clear(&led_strip);
    //led_strip_show(&led_strip);
    // struct led_color_t led_color = {
    //     .red = 0,
    //     .green = 0,
    //     .blue = 0,
    // };

    // for (uint32_t index = 0; index<LED_STRIP_LENGTH ; index++) {
    //         led_strip_get_pixel_color(&led_strip, index, &led_color);
    // }

   led_strip_show(&led_strip);

}

void sendBeam(int distance){
    int lastLED = (int)((float)distance / 3.3);

    struct led_color_t led_color = {
            .red = 200,
            .green = 0,
            .blue = 0,
        };

    eraseLEDStrip();

    uint32_t freestack;
    freestack=uxTaskGetStackHighWaterMark(NULL);
    ESP_LOGI("DISTANCE","Free stack space: %d  ",freestack);
    ESP_LOGI("DISTANCE","lastLED: %d  ",lastLED);

 
    for (uint32_t index=0;index<5;index++){
        led_strip_set_pixel_color(&led_strip, index, &led_color);
    }
    led_strip_show(&led_strip);
    //ESP_LOGI("DISTANCE","Free stack space1: %d  ",freestack);
    //vTaskDelay(pdMS_TO_TICKS(2000));

    for (int i=5; i<lastLED; i++){
        for (uint32_t index = lastLED; index > 0; index--) {
            led_strip_get_pixel_color(&led_strip, (index-1)%LED_STRIP_LENGTH, &led_color);
            led_strip_set_pixel_color(&led_strip, index%LED_STRIP_LENGTH, &led_color);
               //ESP_LOGI("DISTANCE","index: %d  ",index);

        }
           //ESP_LOGI("DISTANCE","LED: %d  ",i);

        led_strip_show(&led_strip);
        vTaskDelay(pdMS_TO_TICKS(10));
    }

   freestack=uxTaskGetStackHighWaterMark(NULL);
    ESP_LOGI("DISTANCE","Free stack space: %d  ",freestack);
    //vTaskDelay(pdMS_TO_TICKS(50));
}

void showPosition(int distance){
    distance = min(330,distance);
    distance = max(10,distance);
    ESP_LOGI("DISTANCE","Show position distance %d",distance);
    int firstLED =  (int)(((float)distance-10.0) / 3.3);
    int lastLED = (int)(((float)distance+10.0) / 3.3);

    struct led_color_t led_color = {
            .red = 200,
            .green = 0,
            .blue = 0,
        };

    //eraseLEDStrip();

    //uint32_t freestack;
    //freestack=uxTaskGetStackHighWaterMark(NULL);
    //ESP_LOGI("DISTANCE","Free stack space: %d  ",freestack);
    ESP_LOGI("DISTANCE","lirstLED: %d  ",firstLED);
    ESP_LOGI("DISTANCE","lastLED: %d  ",lastLED);

 
    for (uint32_t index=firstLED;index<lastLED;index++){
        led_strip_set_pixel_color(&led_strip, index, &led_color);
    }
    led_strip_show(&led_strip);


   //freestack=uxTaskGetStackHighWaterMark(NULL);
    //ESP_LOGI("DISTANCE","Free stack space: %d  ",freestack);
    vTaskDelay(pdMS_TO_TICKS(50));
}


void startBeam(){

    struct led_color_t led_color = {
            .red = 0,
            .green = 200,
            .blue = 0,
        };

     //eraseLEDStrip();
    for (uint32_t index = 0; index<LED_STRIP_LENGTH;index++) {
            led_strip_set_pixel_color(&led_strip, index%LED_STRIP_LENGTH, &led_color);
    }

    led_strip_show(&led_strip);
    vTaskDelay(pdMS_TO_TICKS(500));
    eraseLEDStrip();
}

void stopBeam(){

    struct led_color_t led_color = {
            .red = 200,
            .green = 0,
            .blue = 0,
        };

     //eraseLEDStrip();
    for (uint32_t index = 0; index<LED_STRIP_LENGTH;index++) {
            led_strip_set_pixel_color(&led_strip, index%LED_STRIP_LENGTH, &led_color);
    }


    led_strip_show(&led_strip);
    vTaskDelay(pdMS_TO_TICKS(1000));
    eraseLEDStrip();
    vTaskDelay(pdMS_TO_TICKS(1000));

}

struct detection {
    time_t timestamp;
    int distance;
};

void visualizePositionTask(void *pvParameters)
{
    int lastDistance=0;
    int distance=0;

    while (1)
    {
 
        distance=distanceMeasurementCM();
        if (distance != -1) {
            ESP_LOGI("DISTANCE","Distance: %d\n", distance );
            showPosition(distance);
        }
        
        if (distance>0 && distance!=lastDistance) {
            lastDistance=distance;
            ESP_LOGI("DISTANCE","Timer reset");
            if (xTimerReset(deepSleepTimer,1000U)!=pdPASS){
                 ESP_LOGE("DISTANCE","Timer reset failed."); 
            };

            time_t now = 0;
	        time(&now);
            struct detection event;
            event.timestamp=now;
            event.distance=distance; 
            BaseType_t xStatus = xQueueSendToBack(detections, &event, 0);
            if( xStatus != pdPASS ) {
                ESP_LOGI("DISTANCE", "Queue Detections is full." );
            }

            
        }
        else
            eraseLEDStrip();

        taskYIELD();
        //sendBeam(distance);
        //vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void publishDetectionsTask(void *pvParameters){

    while(true){

        xSemaphoreTake(publishDetectionsSemaphore, portMAX_DELAY);

        int numberEvents=uxQueueMessagesWaiting(detections);

        if (numberEvents>0){
            ESP_LOGI("PUBLISH_DETECTIONS","%d events to publish", numberEvents);  
            wifi_init_sta();
            time_t now1 = 0;
            time(&now1);
            initialize_sntp();
            time_t now2 = 0;
            time(&now2);
            time_t timeshift=now2-now1;
            init_mqtt();

            struct detection event;
            BaseType_t xStatus = xQueueReceive(detections, &event, 0);
            while (xStatus == pdPASS){
                event.timestamp=event.timestamp+timeshift;
                mqttPublish(event.timestamp, event.distance);
                xStatus = xQueueReceive(detections, &event, 0);
            }
        }
        xSemaphoreGive(publishDetectionsSemaphore);
    }
}

void triggerDeepSleep(TimerHandle_t xTimer){
    ESP_LOGI("DISTANCE","Publish detections");   
    //publishDetections();  
    xSemaphoreGive(publishDetectionsSemaphore);
    vTaskDelay(pdMS_TO_TICKS(1000));  
    xSemaphoreTake(publishDetectionsSemaphore, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(5000));  

    stopBeam();
    xSemaphoreTake(led_strip.access_semaphore, portMAX_DELAY);
    ESP_LOGI("DISTANCE","Go into deep sleep"); 

    esp_deep_sleep_start();
}




void app_main()
{

    esp_task_wdt_init(200, false);
 
    detections = xQueueCreate(100, sizeof(struct detection));
    publishDetectionsSemaphore = xSemaphoreCreateBinary();
    if( publishDetectionsSemaphore == NULL ) {
        ESP_LOGE("DISTANCE","publishDetectionsSemaphore could not be created.");    
    }
    ESP_LOGI("DISTANCE","try to take");     
    //xSemaphoreTake(publishDetectionsSemaphore, portMAX_DELAY);
    ESP_LOGI("DISTANCE","taken");     

    nvs_flash_init();
    gpio_install_isr_service(0);

    led_strip.access_semaphore = xSemaphoreCreateBinary();
    bool led_init_ok = led_strip_init(&led_strip);
    assert(led_init_ok);

    startBeam();



    deepSleepTimer=xTimerCreate("DeepSleepTimer",pdMS_TO_TICKS(10000), pdFALSE, NULL, triggerDeepSleep);
    if (deepSleepTimer==NULL){
        ESP_LOGI("DISTANCE","deepSleepTimer is NULL");     
    } else {
        ESP_LOGI("DISTANCE","deepSleepTimer was created");     
        if (xTimerStart(deepSleepTimer, 10000U)==pdPASS){
            ESP_LOGI("DISTANCE","deepSleepTimer was started");     
        };
    }

    #define BUTTON_PIN_BITMASK 0xC00000000
    esp_sleep_enable_ext1_wakeup(BUTTON_PIN_BITMASK,ESP_EXT1_WAKEUP_ANY_HIGH);

    xTaskCreate(visualizePositionTask, "measure_visualize", configMINIMAL_STACK_SIZE * 8, NULL, 1, NULL);
    xTaskCreate(publishDetectionsTask, "publish_detections", configMINIMAL_STACK_SIZE * 8, NULL, 1, NULL);
}
