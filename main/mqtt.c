#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "esp_log.h"
#include "mqtt_client.h"



const static char *TAG = "MQTT";

const static int CONNECTED_BIT = BIT0;
esp_mqtt_client_handle_t mqtt_client;
EventGroupHandle_t mqtt_event_group;


static char *expected_data = NULL;
static char *actual_data = NULL;
static size_t expected_size = 0;
static size_t expected_published = 0;
static size_t actual_published = 0;


#if CONFIG_EXAMPLE_BROKER_CERTIFICATE_OVERRIDDEN == 1
static const uint8_t mqtt_eclipse_org_pem_start[]  = "-----BEGIN CERTIFICATE-----\n" CONFIG_EXAMPLE_BROKER_CERTIFICATE_OVERRIDE "\n-----END CERTIFICATE-----";
#else
extern const uint8_t mqtt_eclipse_org_pem_start[]   asm("_binary_mqtt_eclipse_org_pem_start");
#endif
extern const uint8_t mqtt_eclipse_org_pem_end[]   asm("_binary_mqtt_eclipse_org_pem_end");

static esp_err_t mqtt_event_handler(esp_mqtt_event_handle_t event)
{
    esp_mqtt_client_handle_t client = event->client;
    static int msg_id = 0;
    static int actual_len = 0;
    // your_context_t *context = event->context;
    switch (event->event_id) {
    case MQTT_EVENT_CONNECTED:
		ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
		xEventGroupSetBits(mqtt_event_group, CONNECTED_BIT);
		//msg_id = esp_mqtt_client_subscribe(client, CONFIG_EXAMPLE_SUBSCIBE_TOPIC, qos_test);
        //ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
		esp_restart();
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);
        printf("ID=%d, total_len=%d, data_len=%d, current_data_offset=%d\n", event->msg_id, event->total_data_len, event->data_len, event->current_data_offset);
        if (event->topic) {
            actual_len = event->data_len;
            msg_id = event->msg_id;
        } else {
            actual_len += event->data_len;
            // check consisency with msg_id across multiple data events for single msg
            if (msg_id != event->msg_id) {
                ESP_LOGI(TAG, "Wrong msg_id in chunked message %d != %d", msg_id, event->msg_id);
                abort();
            }
        }
        memcpy(actual_data + event->current_data_offset, event->data, event->data_len);
        if (actual_len == event->total_data_len) {
            if (0 == memcmp(actual_data, expected_data, expected_size)) {
                printf("OK!");
                memset(actual_data, 0, expected_size);
                actual_published ++;
                if (actual_published == expected_published) {
                    printf("Correct pattern received exactly x times\n");
                    ESP_LOGI(TAG, "Test finished correctly!");
                }
            } else {
                printf("FAILED!");
                abort();
            }
        }
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        break;
	case MQTT_EVENT_BEFORE_CONNECT:
        ESP_LOGI(TAG, "MQTT_EVENT_BEFORE_CONNECT");
        break;
	default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
    return ESP_OK;
}

void mqttPublish(time_t timestamp, int distance){
    char msg[256];
    static time_t lastTimestamp=0;
    static time_t increment=0;

    if (lastTimestamp!=timestamp){
        lastTimestamp=timestamp;
        increment=0;
    }
    uint64_t modifiedTimestamp=timestamp;
    increment+=100;
    ESP_LOGI("PUBLISH_DETECTIONS", "timestamp %llu increment %lu",modifiedTimestamp,increment);
    modifiedTimestamp=modifiedTimestamp*1000+increment;
    ESP_LOGI("PUBLISH_DETECTIONS", "timestamp %llu",modifiedTimestamp);

    //sprintf(msg, "{\"timestamp\":%lu000,\"distance\":%d}",timestamp,distance);
	sprintf(msg, "{\"payload_fields\": {\"username\":\"gerndt\",\"device_id\":\"144\",\"timestamp\":%llu,\"distance\":%d}}",modifiedTimestamp,distance);
	ESP_LOGI("PUBLISH_DETECTIONS", "Topic %s: %s\n", "I10_Cray_Board", msg);
    int msg_id = esp_mqtt_client_publish(mqtt_client,"I10_Cray_Board", msg, strlen(msg), 0, 0);

	if (msg_id==-1){
		ESP_LOGE("PUBLISH_DETECTIONS", "msg_id returned by publish is -1!\n");
	}
}

void init_mqtt(void){
	mqtt_event_group = xEventGroupCreate();
	const esp_mqtt_client_config_t mqtt_cfg = {
		.event_handle = mqtt_event_handler,
		.host = "test.mosquitto.org",
		.port = 1883
	};
	
	
	mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
	esp_mqtt_client_start(mqtt_client);
	xEventGroupWaitBits(mqtt_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);
	
}


