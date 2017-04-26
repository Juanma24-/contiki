/*
 * Copyright (c) 2014, Texas Instruments Incorporated - http://www.ti.com/
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/**
 * \addtogroup cc26xx-examples
 * @{
 *
 * \defgroup cc26xx-web-demo CC26xx Web Demo
 * @{
 *
 * \file
 *   Main header file for the CC26XX web demo.
 */
/*---------------------------------------------------------------------------*/
#ifndef ASLTOM_MQTT_IOT_H_
#define ASLTOM_MQTT_IOT_H_
/*---------------------------------------------------------------------------*/
#include "dev/leds.h"
#include "sys/process.h"
#include "mqtt-client.h"

#include <stdint.h>

/*---------------------------------------------------------------------------*/
/* Active probing of RSSI from our preferred parent */

#define ALSTOM_MQTT_IOT_RSSI_MEASURE_INTERVAL_MAX 86400 /* secs: 1 day */
#define ALSTOM_MQTT_IOT_RSSI_MEASURE_INTERVAL_MIN     5 /* secs */
/*---------------------------------------------------------------------------*/
/* User configuration */
/* Take a sensor reading on button press */
#define ALSTOM_MQTT_IOT_SENSOR_READING_TRIGGER &button_left_sensor
#define ALSTOM_MQTT_IOT_OP_LED_OFF &button_right_sensor

/* Payload length of ICMPv6 echo requests used to measure RSSI with def rt */
#define ALSTOM_MQTT_IOT_ECHO_REQ_PAYLOAD_LEN   20

/* Force an MQTT publish on sensor event */
#define ALSTOM_MQTT_IOT_MQTT_PUBLISH_TRIGGER &reed_relay_sensor


#define ALSTOM_MQTT_IOT_STATUS_LED LEDS_GREEN
/*---------------------------------------------------------------------------*/
/* A timeout used when waiting to connect to a network */
#define ALSTOM_MQTT_IOT_NET_CONNECT_PERIODIC        (CLOCK_SECOND >> 3)
/*---------------------------------------------------------------------------*/
/* Default configuration values */
#define ALSTOM_MQTT_IOT_DEFAULT_USERNAME_ID         "User"
#define ALSTOM_MQTT_IOT_DEFAULT_AUTH_TOKEN			    "Pass"
#define ALSTOM_MQTT_IOT_DEFAULT_EVENT_TYPE_ID       "status"
#define ALSTOM_MQTT_IOT_DEFAULT_SUBSCRIBE_CMD_TYPE  "+"
#define ALSTOM_MQTT_IOT_DEFAULT_SALA                "A01"
#define ALSTOM_MQTT_IOT_DEFAULT_TIPO_OP             "PIN01"  
#define ALSTOM_MQTT_IOT_DEFAULT_BROKER_PORT         1883
#define ALSTOM_MQTT_IOT_DEFAULT_PUBLISH_INTERVAL    (30 * CLOCK_SECOND)
#define ALSTOM_MQTT_IOT_DEFAULT_KEEP_ALIVE_TIMER    60
#define ALSTOM_MQTT_IOT_DEFAULT_RSSI_MEAS_INTERVAL  (CLOCK_SECOND * 30)
/*---------------------------------------------------------------------------*/
/*
 * You normally won't have to change anything from here onwards unless you are
 * extending the example
 */
/*---------------------------------------------------------------------------*/
/* Sensor types */
#define ALSTOM_MQTT_IOT_SENSOR_BATMON_TEMP   0
#define ALSTOM_MQTT_IOT_SENSOR_BATMON_VOLT   1
#define ALSTOM_MQTT_IOT_SENSOR_BMP_PRES      2
#define ALSTOM_MQTT_IOT_SENSOR_BMP_TEMP      3
#define ALSTOM_MQTT_IOT_SENSOR_TMP_AMBIENT   4
#define ALSTOM_MQTT_IOT_SENSOR_TMP_OBJECT    5
#define ALSTOM_MQTT_IOT_SENSOR_HDC_TEMP      6
#define ALSTOM_MQTT_IOT_SENSOR_HDC_HUMIDITY  7
#define ALSTOM_MQTT_IOT_SENSOR_OPT_LIGHT     8
#define ALSTOM_MQTT_IOT_SENSOR_MPU_ACC_X     9
#define ALSTOM_MQTT_IOT_SENSOR_MPU_ACC_Y     10
#define ALSTOM_MQTT_IOT_SENSOR_MPU_ACC_Z     11
#define ALSTOM_MQTT_IOT_SENSOR_MPU_GYRO_X    12
#define ALSTOM_MQTT_IOT_SENSOR_MPU_GYRO_Y    13
#define ALSTOM_MQTT_IOT_SENSOR_MPU_GYRO_Z    14
/*---------------------------------------------------------------------------*/
extern process_event_t alstom_mqtt_iot_publish_event;
extern process_event_t alstom_mqtt_iot_config_loaded_event;
extern process_event_t alstom_mqtt_iot_load_config_defaults;
/*---------------------------------------------------------------------------*/
#define ALSTOM_MQTT_IOT_UNIT_TEMP     "C"
#define ALSTOM_MQTT_IOT_UNIT_VOLT     "mV"
#define ALSTOM_MQTT_IOT_UNIT_PRES     "hPa"
#define ALSTOM_MQTT_IOT_UNIT_HUMIDITY "RH"   //%
#define ALSTOM_MQTT_IOT_UNIT_LIGHT    "lux"
#define ALSTOM_MQTT_IOT_UNIT_ACC      "G"
#define ALSTOM_MQTT_IOT_UNIT_GYRO     "deg per sec"
/*---------------------------------------------------------------------------*/
/* A data type for sensor readings, internally stored in a linked list */
#define ALSTOM_MQTT_IOT_CONVERTED_LEN        12

typedef struct alstom_mqtt_iot_sensor_reading {
  struct alstom_mqtt_iot_sensor_reading *next;
  int raw;
  int last;
  const char *descr;
  const char *xml_element;
  const char *form_field;
  char *units;
  uint8_t type;
  uint8_t publish;
  uint8_t changed;
  char converted[ALSTOM_MQTT_IOT_CONVERTED_LEN];
} alstom_mqtt_iot_sensor_reading_t;
/*---------------------------------------------------------------------------*/
/* Global configuration */
typedef struct alstom_mqtt_iot_config_s {
  uint32_t magic;
  int len;
  uint32_t sensors_bitmap;
  int def_rt_ping_interval;
  mqtt_client_config_t mqtt_config;
} alstom_mqtt_iot_config_t;

extern alstom_mqtt_iot_config_t alstom_mqtt_iot_config;
/*---------------------------------------------------------------------------*/
/**
 * \brief Performs a lookup for a reading of a specific type of sensor
 * \param sens_type CC26XX_WEB_DEMO_SENSOR_BATMON_TEMP...
 * \return A pointer to the reading data structure or NULL
 */
const alstom_mqtt_iot_sensor_reading_t *alstom_mqtt_iot_sensor_lookup(int sens_type);

/**
 * \brief Returns the first available sensor reading
 * \return A pointer to the reading data structure or NULL
 */
const alstom_mqtt_iot_sensor_reading_t *alstom_mqtt_iot_sensor_first(void);

/**
 * \brief Print an IPv6 address into a buffer
 * \param buf A pointer to the buffer where this function will print the IPv6
 *        address
 * \param buf_len the length of the buffer
 * \param addr A pointer to the IPv6 address
 * \return The number of bytes written to the buffer
 *
 * It is the caller's responsibility to allocate enough space for buf
 */
int alstom_mqtt_iot_ipaddr_sprintf(char *buf, uint8_t buf_len,
                                   const uip_ipaddr_t *addr);

/**
 * \brief Resets the example to a default configuration
 */
void alstom_mqtt_iot_restore_defaults(void);
/*---------------------------------------------------------------------------*/
#endif /* ASLTOM_MQTT_IOT_H_ */
/*---------------------------------------------------------------------------*/
/**
 * @}
 * @}
 */
