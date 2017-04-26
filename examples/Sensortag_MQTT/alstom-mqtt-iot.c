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
 * ticks + a random interval between 0 and SENSOR_READING_RANDOM ticks
 */
#define SENSOR_READING_PERIOD (CLOCK_SECOND * 20)
#define SENSOR_READING_RANDOM (CLOCK_SECOND << 4)

struct ctimer batmon_timer;

struct ctimer bmp_timer, hdc_timer, tmp_timer, opt_timer, mpu_timer;
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

/* Sensortag sensors */
DEMO_SENSOR(bmp_pres, ALSTOM_MQTT_IOT_SENSOR_BMP_PRES,
            "Air Pressure", "air-pressure", "bmp_pres",
            ALSTOM_MQTT_IOT_UNIT_PRES);
DEMO_SENSOR(bmp_temp, ALSTOM_MQTT_IOT_SENSOR_BMP_TEMP,
            "Air Temp", "air-temp", "bmp_temp",
            ALSTOM_MQTT_IOT_UNIT_TEMP);
DEMO_SENSOR(hdc_temp, ALSTOM_MQTT_IOT_SENSOR_HDC_TEMP,
            "HDC Temp", "hdc-temp", "hdc_temp",
            ALSTOM_MQTT_IOT_UNIT_TEMP);
DEMO_SENSOR(hdc_hum, ALSTOM_MQTT_IOT_SENSOR_HDC_HUMIDITY,
            "HDC Humidity", "hdc-humidity", "hdc_hum",
            ALSTOM_MQTT_IOT_UNIT_HUMIDITY);
DEMO_SENSOR(tmp_amb, ALSTOM_MQTT_IOT_SENSOR_TMP_AMBIENT,
            "Ambient Temp", "ambient-temp", "tmp_amb",
            ALSTOM_MQTT_IOT_UNIT_TEMP);
DEMO_SENSOR(tmp_obj, ALSTOM_MQTT_IOT_SENSOR_TMP_OBJECT,
            "Object Temp", "object-temp", "tmp_obj",
            ALSTOM_MQTT_IOT_UNIT_TEMP);
DEMO_SENSOR(opt, ALSTOM_MQTT_IOT_SENSOR_OPT_LIGHT,
            "Light", "light", "light",
            ALSTOM_MQTT_IOT_UNIT_LIGHT);

/* MPU Readings */
DEMO_SENSOR(mpu_acc_x, ALSTOM_MQTT_IOT_SENSOR_MPU_ACC_X,
            "Acc X", "acc-x", "acc_x",
            ALSTOM_MQTT_IOT_UNIT_ACC);
DEMO_SENSOR(mpu_acc_y, ALSTOM_MQTT_IOT_SENSOR_MPU_ACC_Y,
            "Acc Y", "acc-y", "acc_y",
            ALSTOM_MQTT_IOT_UNIT_ACC);
DEMO_SENSOR(mpu_acc_z, ALSTOM_MQTT_IOT_SENSOR_MPU_ACC_Z,
            "Acc Z", "acc-z", "acc_z",
            ALSTOM_MQTT_IOT_UNIT_ACC);

DEMO_SENSOR(mpu_gyro_x, ALSTOM_MQTT_IOT_SENSOR_MPU_GYRO_X,
            "Gyro X", "gyro-x", "gyro_x",
            ALSTOM_MQTT_IOT_UNIT_GYRO);
DEMO_SENSOR(mpu_gyro_y, ALSTOM_MQTT_IOT_SENSOR_MPU_GYRO_Y,
            "Gyro Y", "gyro-y", "gyro_y",
            ALSTOM_MQTT_IOT_UNIT_GYRO);
DEMO_SENSOR(mpu_gyro_z, ALSTOM_MQTT_IOT_SENSOR_MPU_GYRO_Z,
            "Gyro Z", "gyro-z", "gyro_Z",
            ALSTOM_MQTT_IOT_UNIT_GYRO);
/*---------------------------------------------------------------------------*/
static void init_bmp_reading(void *data);
static void init_light_reading(void *data);
static void init_hdc_reading(void *data);
static void init_tmp_reading(void *data);
static void init_mpu_reading(void *data);
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
  alstom_mqtt_iot_sensor_reading_t *reading = NULL;

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
    alstom_mqtt_iot_config.sensors_bitmap = 0;

    for(reading = list_head(sensor_list);reading != NULL;reading = list_item_next(reading)) {
      if(reading->publish) {
        alstom_mqtt_iot_config.sensors_bitmap |= (1 << reading->type);
      }
    }

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
  alstom_mqtt_iot_sensor_reading_t *reading = NULL;

  int rv = ext_flash_open();

  if(!rv) {
    printf("Could not open flash to load config\n");
    ext_flash_close();
    return;
  }

  rv = ext_flash_read(CONFIG_FLASH_OFFSET, sizeof(tmp_cfg),
                      (uint8_t *)&tmp_cfg);

  ext_flash_close();

  if(!rv) {
    printf("Error loading config\n");
    return;
  }

  if(tmp_cfg.magic == CONFIG_MAGIC && tmp_cfg.len == sizeof(tmp_cfg)) {
    memcpy(&alstom_mqtt_iot_config, &tmp_cfg, sizeof(alstom_mqtt_iot_config));
  }

  for(reading = list_head(sensor_list);
      reading != NULL;
      reading = list_item_next(reading)) {
    if(alstom_mqtt_iot_config.sensors_bitmap & (1 << reading->type)) {
      reading->publish = 1;
    } else {
      reading->publish = 0;
      snprintf(reading->converted, ALSTOM_MQTT_IOT_CONVERTED_LEN, "\"N/A\"");
    }
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
  clock_time_t next = SENSOR_READING_PERIOD +
    (random_rand() % SENSOR_READING_RANDOM);

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
compare_and_update(alstom_mqtt_iot_sensor_reading_t *reading)
{
  if(reading->last == reading->raw) {
    reading->changed = 0;
  } else {
    reading->last = reading->raw;
    reading->changed = 1;
  }
}
/*---------------------------------------------------------------------------*/
static void
print_mpu_reading(int reading, char *buf)
{
  char *loc_buf = buf;

  if(reading < 0) {
    sprintf(loc_buf, "-");
    reading = -reading;
    loc_buf++;
  }

  sprintf(loc_buf, "%d.%02d", reading / 100, reading % 100);
}
/*---------------------------------------------------------------------------*/
static void
get_bmp_reading()
{
  int value;
  char *buf;
  clock_time_t next = SENSOR_READING_PERIOD +
    (random_rand() % SENSOR_READING_RANDOM);

  if(bmp_pres_reading.publish) {
    value = bmp_280_sensor.value(BMP_280_SENSOR_TYPE_PRESS);
    if(value != CC26XX_SENSOR_READING_ERROR) {
      bmp_pres_reading.raw = value;

      compare_and_update(&bmp_pres_reading);

      buf = bmp_pres_reading.converted;
      memset(buf, 0, ALSTOM_MQTT_IOT_CONVERTED_LEN);
      snprintf(buf, ALSTOM_MQTT_IOT_CONVERTED_LEN, "%d.%02d", value / 100,
               value % 100);
    }
  }

  if(bmp_temp_reading.publish) {
    value = bmp_280_sensor.value(BMP_280_SENSOR_TYPE_TEMP);
    if(value != CC26XX_SENSOR_READING_ERROR) {
      bmp_temp_reading.raw = value;

      compare_and_update(&bmp_temp_reading);

      buf = bmp_temp_reading.converted;
      memset(buf, 0, ALSTOM_MQTT_IOT_CONVERTED_LEN);
      snprintf(buf, ALSTOM_MQTT_IOT_CONVERTED_LEN, "%d.%02d", value / 100,
               value % 100);
    }
  }

  SENSORS_DEACTIVATE(bmp_280_sensor);

  ctimer_set(&bmp_timer, next, init_bmp_reading, NULL);
}
/*---------------------------------------------------------------------------*/
static void
get_tmp_reading()
{
  int value;
  char *buf;
  clock_time_t next = SENSOR_READING_PERIOD +
    (random_rand() % SENSOR_READING_RANDOM);

  if(tmp_amb_reading.publish || tmp_obj_reading.publish) {
    if(tmp_007_sensor.value(TMP_007_SENSOR_TYPE_ALL) ==
       CC26XX_SENSOR_READING_ERROR) {

      SENSORS_DEACTIVATE(tmp_007_sensor);
      ctimer_set(&tmp_timer, next, init_tmp_reading, NULL);
    }
  }

  if(tmp_amb_reading.publish) {
    value = tmp_007_sensor.value(TMP_007_SENSOR_TYPE_AMBIENT);
    tmp_amb_reading.raw = value;

    compare_and_update(&tmp_amb_reading);

    buf = tmp_amb_reading.converted;
    memset(buf, 0, ALSTOM_MQTT_IOT_CONVERTED_LEN);
    snprintf(buf, ALSTOM_MQTT_IOT_CONVERTED_LEN, "%d.%03d", value / 1000,
             value % 1000);
  }

  if(tmp_obj_reading.publish) {
    value = tmp_007_sensor.value(TMP_007_SENSOR_TYPE_OBJECT);
    tmp_obj_reading.raw = value;

    compare_and_update(&tmp_obj_reading);

    buf = tmp_obj_reading.converted;
    memset(buf, 0, ALSTOM_MQTT_IOT_CONVERTED_LEN);
    snprintf(buf, ALSTOM_MQTT_IOT_CONVERTED_LEN, "%d.%03d", value / 1000,
             value % 1000);
  }

  SENSORS_DEACTIVATE(tmp_007_sensor);

  ctimer_set(&tmp_timer, next, init_tmp_reading, NULL);
}
/*---------------------------------------------------------------------------*/
static void
get_hdc_reading()
{
  int value;
  char *buf;
  clock_time_t next = SENSOR_READING_PERIOD +
    (random_rand() % SENSOR_READING_RANDOM);

  if(hdc_temp_reading.publish) {
    value = hdc_1000_sensor.value(HDC_1000_SENSOR_TYPE_TEMP);
    if(value != CC26XX_SENSOR_READING_ERROR) {
      hdc_temp_reading.raw = value;

      compare_and_update(&hdc_temp_reading);

      buf = hdc_temp_reading.converted;
      memset(buf, 0, ALSTOM_MQTT_IOT_CONVERTED_LEN);
      snprintf(buf, ALSTOM_MQTT_IOT_CONVERTED_LEN, "%d.%02d", value / 100,
               value % 100);
    }
  }

  if(hdc_hum_reading.publish) {
    value = hdc_1000_sensor.value(HDC_1000_SENSOR_TYPE_HUMIDITY);
    if(value != CC26XX_SENSOR_READING_ERROR) {
      hdc_hum_reading.raw = value;

      compare_and_update(&hdc_hum_reading);

      buf = hdc_hum_reading.converted;
      memset(buf, 0, ALSTOM_MQTT_IOT_CONVERTED_LEN);
      snprintf(buf, ALSTOM_MQTT_IOT_CONVERTED_LEN, "%d.%02d", value / 100,
               value % 100);
    }
  }

  ctimer_set(&hdc_timer, next, init_hdc_reading, NULL);
}
/*---------------------------------------------------------------------------*/
static void
get_light_reading()
{
  int value;
  char *buf;
  clock_time_t next = SENSOR_READING_PERIOD +
    (random_rand() % SENSOR_READING_RANDOM);

  value = opt_3001_sensor.value(0);

  if(value != CC26XX_SENSOR_READING_ERROR) {
    opt_reading.raw = value;

    compare_and_update(&opt_reading);

    buf = opt_reading.converted;
    memset(buf, 0, ALSTOM_MQTT_IOT_CONVERTED_LEN);
    snprintf(buf, ALSTOM_MQTT_IOT_CONVERTED_LEN, "%d.%02d", value / 100,
             value % 100);
  }

  /* The OPT will turn itself off, so we don't need to call its DEACTIVATE */
  ctimer_set(&opt_timer, next, init_light_reading, NULL);
}
/*---------------------------------------------------------------------------*/
static void
get_mpu_reading()
{
  clock_time_t next = SENSOR_READING_PERIOD +
    (random_rand() % SENSOR_READING_RANDOM);
  int raw;

  if(mpu_gyro_x_reading.publish) {
    raw = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_GYRO_X);
    if(raw != CC26XX_SENSOR_READING_ERROR) {
      mpu_gyro_x_reading.raw = raw;
    }
  }

  if(mpu_gyro_y_reading.publish) {
    raw = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_GYRO_Y);
    if(raw != CC26XX_SENSOR_READING_ERROR) {
      mpu_gyro_y_reading.raw = raw;
    }
  }

  if(mpu_gyro_z_reading.publish) {
    raw = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_GYRO_Z);
    if(raw != CC26XX_SENSOR_READING_ERROR) {
      mpu_gyro_z_reading.raw = raw;
    }
  }

  if(mpu_acc_x_reading.publish) {
    raw = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_X);
    if(raw != CC26XX_SENSOR_READING_ERROR) {
      mpu_acc_x_reading.raw = raw;
    }
  }

  if(mpu_acc_y_reading.publish) {
    raw = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_Y);
    if(raw != CC26XX_SENSOR_READING_ERROR) {
      mpu_acc_y_reading.raw = raw;
    }
  }

  if(mpu_acc_z_reading.publish) {
    raw = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_Z);
    if(raw != CC26XX_SENSOR_READING_ERROR) {
      mpu_acc_z_reading.raw = raw;
    }
  }

  SENSORS_DEACTIVATE(mpu_9250_sensor);

  if(mpu_gyro_x_reading.publish) {
    compare_and_update(&mpu_gyro_x_reading);
    memset(mpu_gyro_x_reading.converted, 0, ALSTOM_MQTT_IOT_CONVERTED_LEN);
    print_mpu_reading(mpu_gyro_x_reading.raw, mpu_gyro_x_reading.converted);
  }

  if(mpu_gyro_y_reading.publish) {
    compare_and_update(&mpu_gyro_y_reading);
    memset(mpu_gyro_y_reading.converted, 0, ALSTOM_MQTT_IOT_CONVERTED_LEN);
    print_mpu_reading(mpu_gyro_y_reading.raw, mpu_gyro_y_reading.converted);
  }

  if(mpu_gyro_z_reading.publish) {
    compare_and_update(&mpu_gyro_z_reading);
    memset(mpu_gyro_z_reading.converted, 0, ALSTOM_MQTT_IOT_CONVERTED_LEN);
    print_mpu_reading(mpu_gyro_z_reading.raw, mpu_gyro_z_reading.converted);
  }

  if(mpu_acc_x_reading.publish) {
    compare_and_update(&mpu_acc_x_reading);
    memset(mpu_acc_x_reading.converted, 0, ALSTOM_MQTT_IOT_CONVERTED_LEN);
    print_mpu_reading(mpu_acc_x_reading.raw, mpu_acc_x_reading.converted);
  }

  if(mpu_acc_y_reading.publish) {
    compare_and_update(&mpu_acc_y_reading);
    memset(mpu_acc_y_reading.converted, 0, ALSTOM_MQTT_IOT_CONVERTED_LEN);
    print_mpu_reading(mpu_acc_y_reading.raw, mpu_acc_y_reading.converted);
  }

  if(mpu_acc_z_reading.publish) {
    compare_and_update(&mpu_acc_z_reading);
    memset(mpu_acc_z_reading.converted, 0, ALSTOM_MQTT_IOT_CONVERTED_LEN);
    print_mpu_reading(mpu_acc_z_reading.raw, mpu_acc_z_reading.converted);
  }

  /* We only use the single timer */
  ctimer_set(&mpu_timer, next, init_mpu_reading, NULL);
}
/*---------------------------------------------------------------------------*/
static void
init_tmp_reading(void *data)
{
  if(tmp_amb_reading.publish || tmp_obj_reading.publish) {
    SENSORS_ACTIVATE(tmp_007_sensor);
  } else {
    ctimer_set(&tmp_timer, CLOCK_SECOND, init_tmp_reading, NULL);
  }
}
/*---------------------------------------------------------------------------*/
static void
init_bmp_reading(void *data)
{
  if(bmp_pres_reading.publish || bmp_temp_reading.publish) {
    SENSORS_ACTIVATE(bmp_280_sensor);
  } else {
    ctimer_set(&bmp_timer, CLOCK_SECOND, init_bmp_reading, NULL);
  }
}
/*---------------------------------------------------------------------------*/
static void
init_hdc_reading(void *data)
{
  if(hdc_hum_reading.publish || hdc_temp_reading.publish) {
    SENSORS_ACTIVATE(hdc_1000_sensor);
  } else {
    ctimer_set(&hdc_timer, CLOCK_SECOND, init_hdc_reading, NULL);
  }
}
/*---------------------------------------------------------------------------*/
static void
init_light_reading(void *data)
{
  if(opt_reading.publish) {
    SENSORS_ACTIVATE(opt_3001_sensor);
  } else {
    ctimer_set(&opt_timer, CLOCK_SECOND, init_light_reading, NULL);
  }
}
/*---------------------------------------------------------------------------*/
static void
init_mpu_reading(void *data)
{
  int readings_bitmap = 0;

  if(mpu_acc_x_reading.publish || mpu_acc_y_reading.publish ||
     mpu_acc_z_reading.publish) {
    readings_bitmap |= MPU_9250_SENSOR_TYPE_ACC;
  }

  if(mpu_gyro_x_reading.publish || mpu_gyro_y_reading.publish ||
     mpu_gyro_z_reading.publish) {
    readings_bitmap |= MPU_9250_SENSOR_TYPE_GYRO;
  }

  if(readings_bitmap) {
    mpu_9250_sensor.configure(SENSORS_ACTIVE, readings_bitmap);
  } else {
    ctimer_set(&mpu_timer, CLOCK_SECOND, init_mpu_reading, NULL);
  }
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

  init_bmp_reading(NULL);
  init_light_reading(NULL);
  init_hdc_reading(NULL);
  init_tmp_reading(NULL);
  init_mpu_reading(NULL);

  return;
}
/*---------------------------------------------------------------------------*/
static void
init_sensors(void)
{

  list_add(sensor_list, &batmon_temp_reading);
  list_add(sensor_list, &batmon_volt_reading);
  SENSORS_ACTIVATE(batmon_sensor);

  list_add(sensor_list, &bmp_pres_reading);
  list_add(sensor_list, &bmp_temp_reading);

  list_add(sensor_list, &tmp_obj_reading);
  list_add(sensor_list, &tmp_amb_reading);

  list_add(sensor_list, &opt_reading);

  list_add(sensor_list, &hdc_hum_reading);
  list_add(sensor_list, &hdc_temp_reading);

  list_add(sensor_list, &mpu_acc_x_reading);
  list_add(sensor_list, &mpu_acc_y_reading);
  list_add(sensor_list, &mpu_acc_z_reading);
  list_add(sensor_list, &mpu_gyro_x_reading);
  list_add(sensor_list, &mpu_gyro_y_reading);
  list_add(sensor_list, &mpu_gyro_z_reading);

  SENSORS_ACTIVATE(reed_relay_sensor);
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
  alstom_mqtt_iot_config.sensors_bitmap = 0xFFFFFFFF; /* all on by default */
  alstom_mqtt_iot_config.def_rt_ping_interval = ALSTOM_MQTT_IOT_DEFAULT_RSSI_MEAS_INTERVAL;
  load_config();                                                                  //Carga la configuración

  /*
   * Notify all other processes (basically the ones in this demo) that the
   * configuration has been loaded from flash, in case they care
   */
  process_post(PROCESS_BROADCAST, alstom_mqtt_iot_config_loaded_event, NULL);

  init_sensor_readings();                                                                                     //Inicia lectura de Sensores

  def_rt_rssi = 0x8000000;
  uip_icmp6_echo_reply_callback_add(&echo_reply_notification,
                                    echo_reply_handler);
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
    } else if(ev == sensors_event && data == &bmp_280_sensor) {
      get_bmp_reading();
    } else if(ev == sensors_event && data == &opt_3001_sensor) {
      get_light_reading();
    } else if(ev == sensors_event && data == &hdc_1000_sensor) {
      get_hdc_reading();
    } else if(ev == sensors_event && data == &tmp_007_sensor) {
      get_tmp_reading();
    } else if(ev == sensors_event && data == &mpu_9250_sensor) {
      get_mpu_reading();
    }

    PROCESS_YIELD();
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
/**
 * @}
 */
