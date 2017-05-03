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
 * \addtogroup cc26xx-web-demo
 * @{
 *
 * \file
 *   Main module for the CC26XX web demo. Activates on-device resources,
 *   takes sensor readings periodically and caches them for all other modules
 *   to use.
 */
/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "contiki-net.h"
#include "board-peripherals.h"
#include "lib/sensors.h"
#include "lib/list.h"
#include "sys/process.h"
#include "net/ipv6/sicslowpan.h"
#include "button-sensor.h"
#include "batmon-sensor.h"
#include "alstom-mqtt-iot.h"
#include "mqtt-client.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/*---------------------------------------------------------------------------*/
PROCESS_NAME(cetic_6lbr_client_process);
PROCESS(alstom_mqtt_iot_process, "CC26XX Web Demo");
/*---------------------------------------------------------------------------*/
/*
 * Update sensor readings in a staggered fashion every SENSOR_READING_PERIOD
 * ticks
 */
#define SENSOR_READING_PERIOD (CLOCK_SECOND * 20)

struct ctimer batmon_timer;

/*---------------------------------------------------------------------------*/
/* Provide visible feedback via LEDS while searching for a network */
#define NO_NET_LED_DURATION        (ALSTOM_MQTT_IOT_NET_CONNECT_PERIODIC >> 1)

static struct etimer et;
static struct ctimer ct;
/*---------------------------------------------------------------------------*/
/* Parent RSSI functionality */

static struct uip_icmp6_echo_reply_notification echo_reply_notification;
static struct etimer echo_request_timer;
int def_rt_rssi = 0;

/*---------------------------------------------------------------------------*/
process_event_t alstom_mqtt_iot_publish_event;
process_event_t alstom_mqtt_iot_config_loaded_event;
process_event_t alstom_mqtt_iot_load_config_defaults;
/*---------------------------------------------------------------------------*/
/* Saved settings on flash: store, offset, magic */
#define CONFIG_FLASH_OFFSET        0
#define CONFIG_MAGIC      0xCC265002

alstom_mqtt_iot_config_t alstom_mqtt_iot_config;
/*---------------------------------------------------------------------------*/
/* A cache of sensor values. Updated periodically or upon key press */
LIST(sensor_list);
/*---------------------------------------------------------------------------*/
/* The objects representing sensors used in this demo */
#define DEMO_SENSOR(name, type, descr, xml_element, form_field, units) \
  alstom_mqtt_iot_sensor_reading_t name##_reading = \
  { NULL, 0, 0, descr, xml_element, form_field, units, type, 1, 1 }

/* CC26xx sensors */
DEMO_SENSOR(batmon_temp, ALSTOM_MQTT_IOT_SENSOR_BATMON_TEMP,
            "Battery Temp", "battery-temp", "batmon_temp",
            ALSTOM_MQTT_IOT_UNIT_TEMP);
DEMO_SENSOR(batmon_volt, ALSTOM_MQTT_IOT_SENSOR_BATMON_VOLT,
            "Battery Volt", "battery-volt", "batmon_volt",
            ALSTOM_MQTT_IOT_UNIT_VOLT);
/*---------------------------------------------------------------------------*/
static void
publish_led_off(void *d)
{
  leds_off(ALSTOM_MQTT_IOT_STATUS_LED);
}
/*---------------------------------------------------------------------------*/
static void
save_config()
{
  /* Dump current running config to flash */
  int rv;

  rv = ext_flash_open();

  if(!rv) {
    printf("Could not open flash to save config\n");
    ext_flash_close();
    return;
  }

  rv = ext_flash_erase(CONFIG_FLASH_OFFSET, sizeof(alstom_mqtt_iot_config_t));

  if(!rv) {
    printf("Error erasing flash\n");
  } else {
    alstom_mqtt_iot_config.magic = CONFIG_MAGIC;
    alstom_mqtt_iot_config.len = sizeof(alstom_mqtt_iot_config_t);

    rv = ext_flash_write(CONFIG_FLASH_OFFSET, sizeof(alstom_mqtt_iot_config_t),(uint8_t *)&alstom_mqtt_iot_config);
    if(!rv) {
      printf("Error saving config\n");
    }
  }

  ext_flash_close();
}
/*---------------------------------------------------------------------------*/
static void
load_config()
{
  /* Read from flash into a temp buffer */
  alstom_mqtt_iot_config_t tmp_cfg;
  

  int rv = ext_flash_open();

  if(!rv) {
    printf("Could not open flash to load config\n");
    ext_flash_close();
    return;
  }

  rv = ext_flash_read(CONFIG_FLASH_OFFSET, sizeof(tmp_cfg),(uint8_t *)&tmp_cfg);

  ext_flash_close();

  if(!rv) {
    printf("Error loading config\n");
    return;
  }

  if(tmp_cfg.magic == CONFIG_MAGIC && tmp_cfg.len == sizeof(tmp_cfg)) {
    memcpy(&alstom_mqtt_iot_config, &tmp_cfg, sizeof(alstom_mqtt_iot_config));
  }
}
/*---------------------------------------------------------------------------*/
/* Don't start everything here, we need to dictate order of initialisation */
AUTOSTART_PROCESSES(&alstom_mqtt_iot_process);
/*---------------------------------------------------------------------------*/
int
alstom_mqtt_iot_ipaddr_sprintf(char *buf, uint8_t buf_len,
                               const uip_ipaddr_t *addr)
{
  uint16_t a;
  uint8_t len = 0;
  int i, f;
  for(i = 0, f = 0; i < sizeof(uip_ipaddr_t); i += 2) {
    a = (addr->u8[i] << 8) + addr->u8[i + 1];
    if(a == 0 && f >= 0) {
      if(f++ == 0) {
        len += snprintf(&buf[len], buf_len - len, "::");
      }
    } else {
      if(f > 0) {
        f = -1;
      } else if(i > 0) {
        len += snprintf(&buf[len], buf_len - len, ":");
      }
      len += snprintf(&buf[len], buf_len - len, "%x", a);
    }
  }

  return len;
}
/*---------------------------------------------------------------------------*/
const alstom_mqtt_iot_sensor_reading_t *
alstom_mqtt_iot_sensor_lookup(int sens_type)
{
  alstom_mqtt_iot_sensor_reading_t *reading = NULL;

  for(reading = list_head(sensor_list);
      reading != NULL;
      reading = list_item_next(reading)) {
    if(reading->type == sens_type) {
      return reading;
    }
  }

  return NULL;
}
/*---------------------------------------------------------------------------*/
const alstom_mqtt_iot_sensor_reading_t *
alstom_mqtt_iot_sensor_first()
{
  return list_head(sensor_list);
}
/*---------------------------------------------------------------------------*/
void
alstom_mqtt_iot_restore_defaults(void)
{
  alstom_mqtt_iot_sensor_reading_t *reading = NULL;

  leds_on(LEDS_ALL);

  for(reading = list_head(sensor_list);reading != NULL;reading = list_item_next(reading)) {
    reading->publish = 1;
  }


  process_post_synch(&mqtt_client_process,alstom_mqtt_iot_load_config_defaults, NULL);

  save_config();

  leds_off(LEDS_ALL);
}
/*---------------------------------------------------------------------------*/

static void
echo_reply_handler(uip_ipaddr_t *source, uint8_t ttl, uint8_t *data,
                   uint16_t datalen)
{
  if(uip_ip6addr_cmp(source, uip_ds6_defrt_choose())) {
    def_rt_rssi = sicslowpan_get_last_rssi();
  }
}
/*---------------------------------------------------------------------------*/
static void
ping_parent(void)
{
  if(uip_ds6_get_global(ADDR_PREFERRED) == NULL) {
    return;
  }

  uip_icmp6_send(uip_ds6_defrt_choose(), ICMP6_ECHO_REQUEST, 0,
                 ALSTOM_MQTT_IOT_ECHO_REQ_PAYLOAD_LEN);
}
/*---------------------------------------------------------------------------*/
static void
get_batmon_reading(void *data)
{
  int value;
  char *buf;
  clock_time_t next = SENSOR_READING_PERIOD;

  if(batmon_temp_reading.publish) {
    value = batmon_sensor.value(BATMON_SENSOR_TYPE_TEMP);
    if(value != CC26XX_SENSOR_READING_ERROR) {
      batmon_temp_reading.raw = value;

      buf = batmon_temp_reading.converted;
      memset(buf, 0, ALSTOM_MQTT_IOT_CONVERTED_LEN);
      snprintf(buf, ALSTOM_MQTT_IOT_CONVERTED_LEN, "%d", value);
    }
  }

  if(batmon_volt_reading.publish) {
    value = batmon_sensor.value(BATMON_SENSOR_TYPE_VOLT);
    if(value != CC26XX_SENSOR_READING_ERROR) {
      batmon_volt_reading.raw = value;

      buf = batmon_volt_reading.converted;
      memset(buf, 0, ALSTOM_MQTT_IOT_CONVERTED_LEN);
      snprintf(buf, ALSTOM_MQTT_IOT_CONVERTED_LEN, "%d", (value * 125) >> 5);
    }
  }

  ctimer_set(&batmon_timer, next, get_batmon_reading, NULL);
}
/*---------------------------------------------------------------------------*/
static void
init_sensor_readings(void)
{
  /*
   * Make a first pass and get all initial sensor readings. This will also
   * trigger periodic value updates
   */
  get_batmon_reading(NULL);
  return;
}
/*---------------------------------------------------------------------------*/
static void
init_sensors(void)
{

  list_add(sensor_list, &batmon_temp_reading);
  list_add(sensor_list, &batmon_volt_reading);
  SENSORS_ACTIVATE(batmon_sensor);
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(alstom_mqtt_iot_process, ev, data)
{
  PROCESS_BEGIN();

  printf("ALSTOM MQTT IoT Process\n");

  
  init_sensors();                                                                                                                   //Inicia Sensores
                                                                                                                                    //Reservas de memoria para procesos
  alstom_mqtt_iot_publish_event = process_alloc_event();
  alstom_mqtt_iot_config_loaded_event = process_alloc_event();
  alstom_mqtt_iot_load_config_defaults = process_alloc_event();

  process_start(&mqtt_client_process, NULL);                                                                                        //Inicia el cliente MQTT

  /*
   * Now that processes have set their own config default values, set our
   * own defaults and restore saved config from flash...
   */
  alstom_mqtt_iot_config.def_rt_ping_interval = ALSTOM_MQTT_IOT_DEFAULT_RSSI_MEAS_INTERVAL;
  load_config();                                                                  //Carga la configuración

  /*
   * Notify all other processes (basically the ones in this demo) that the
   * configuration has been loaded from flash, in case they care
   */
  process_post(PROCESS_BROADCAST, alstom_mqtt_iot_config_loaded_event, NULL);

  init_sensor_readings();

  def_rt_rssi = 0x8000000;
  uip_icmp6_echo_reply_callback_add(&echo_reply_notification,echo_reply_handler);
  etimer_set(&echo_request_timer, ALSTOM_MQTT_IOT_NET_CONNECT_PERIODIC);

  etimer_set(&et, ALSTOM_MQTT_IOT_NET_CONNECT_PERIODIC);

  /*
   * Update all sensor readings on a configurable sensors_event
   * (e.g a button press / or reed trigger)
   */
  while(1) {
    if(ev == PROCESS_EVENT_TIMER && etimer_expired(&et)) {
      if(uip_ds6_get_global(ADDR_PREFERRED) == NULL) {
        leds_on(ALSTOM_MQTT_IOT_STATUS_LED);
        ctimer_set(&ct, NO_NET_LED_DURATION, publish_led_off, NULL);
        etimer_set(&et, ALSTOM_MQTT_IOT_NET_CONNECT_PERIODIC);
      }
    }

    if(ev == PROCESS_EVENT_TIMER && etimer_expired(&echo_request_timer)) {
      if(uip_ds6_get_global(ADDR_PREFERRED) == NULL) {
        etimer_set(&echo_request_timer, ALSTOM_MQTT_IOT_NET_CONNECT_PERIODIC);
      } else {
        ping_parent();
        etimer_set(&echo_request_timer, alstom_mqtt_iot_config.def_rt_ping_interval);
      }
    }

    if(ev == sensors_event && data == ALSTOM_MQTT_IOT_SENSOR_READING_TRIGGER) {
      if((ALSTOM_MQTT_IOT_SENSOR_READING_TRIGGER)->value(BUTTON_SENSOR_VALUE_DURATION) > CLOCK_SECOND * 5) {
        printf("Restoring defaults!\n");
        alstom_mqtt_iot_restore_defaults();
      } else {
        init_sensor_readings();
        process_post(PROCESS_BROADCAST, alstom_mqtt_iot_publish_event, NULL);
      }
    }

    PROCESS_YIELD();
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
/**
 * @}
 */
