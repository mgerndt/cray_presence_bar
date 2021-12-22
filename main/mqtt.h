#ifndef MQTT_H
#define MQTT_H
#include "esp_sntp.h"


void init_mqtt(void);
void mqttPublish(time_t timestamp, int distance);

#endif