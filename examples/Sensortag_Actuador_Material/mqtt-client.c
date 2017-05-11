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
 *   Cliente MQTT para uso como actuador. El Sensortag se subscribe a 4 topic llamados
 *   de actuación porque su función es actuar sobre el led rojo. Estos topics pueden
 *   ser configurados mediante topics de configuracion asociados al id del cliente.
 *   Por último, el Sensortag también publica en un intervalo de baja frecuencia datos
 *   relacionados con su consumo para poder monitorizar el nivel de batería.
 */
/*---------------------------------------------------------------------------*/
#include "contiki-conf.h"
#include "rpl/rpl-private.h"
#include "mqtt.h"
#include "net/rpl/rpl.h"
#include "net/ip/uip.h"
#include "net/ipv6/uip-icmp6.h"
#include "sys/etimer.h"
#include "sys/ctimer.h"
#include "lib/sensors.h"
#include "button-sensor.h"
#include "board-peripherals.h"
#include "alstom-mqtt-iot.h"
#include "dev/leds.h"
#include "mqtt-client.h"

#include <string.h>
#include <strings.h>
/*---------------------------------------------------------------------------*/
/*
 * Dirección IP del Broker. El Gateway utiliza NAT64 por lo que las direcciones
 * IPv4 son mapeadas a direcciones IPv6. La conversion se puede realizar facilmente
 * con alguna herramienta web como: https://www.ultratools.com/tools/ipv4toipv6
 */
/* Dorección IP local del Broker Mosquitto ubicado en Laptop */

static const char *broker_ip = "0000:0000:0000:0000:0000:ffff:c0a8:012f";
/*---------------------------------------------------------------------------*/
/*
 * A timeout used when waiting for something to happen (e.g. to connect or to
 * disconnect)
 */
#define STATE_MACHINE_PERIODIC     (CLOCK_SECOND >> 1)
/*---------------------------------------------------------------------------*/
/* Provide visible feedback via LEDS during various states */
/* When connecting to broker */
#define CONNECTING_LED_DURATION    (CLOCK_SECOND >> 3)

/* Each time we try to publish */
#define PUBLISH_LED_ON_DURATION    (CLOCK_SECOND)
/*---------------------------------------------------------------------------*/
/* Connections and reconnections */
#define RETRY_FOREVER              0xFF
#define RECONNECT_INTERVAL         (CLOCK_SECOND * 2)

/*
 * Number of times to try reconnecting to the broker.
 * Can be a limited number (e.g. 3, 10 etc) or can be set to RETRY_FOREVER
 */
#define RECONNECT_ATTEMPTS         5
#define CONNECTION_STABLE_TIME     (CLOCK_SECOND * 5)
#define NEW_CONFIG_WAIT_INTERVAL   (CLOCK_SECOND * 20)
static struct timer connection_life;
static uint8_t connect_attempt;
/*---------------------------------------------------------------------------*/
/* Estados del cliente MQTT utilizados por la maquina de estados */
static uint8_t state;
#define MQTT_CLIENT_STATE_INIT            0
#define MQTT_CLIENT_STATE_REGISTERED      1
#define MQTT_CLIENT_STATE_CONNECTING      2
#define MQTT_CLIENT_STATE_CONNECTED       3
#define MQTT_CLIENT_STATE_PUBLISHING      4
#define MQTT_CLIENT_STATE_DISCONNECTED    5
#define MQTT_CLIENT_STATE_NEWCONFIG       6
#define MQTT_CLIENT_STATE_CONFIG_ERROR 0xFE
#define MQTT_CLIENT_STATE_ERROR        0xFF
/*---------------------------------------------------------------------------*/
/* Maximum TCP segment size for outgoing segments of our socket */
#define MQTT_CLIENT_MAX_SEGMENT_SIZE    32
/*---------------------------------------------------------------------------*/
/*
 * Buffers for Client ID and Topic.
 * Make sure they are large enough to hold the entire respective string
 *
 * En este caso se utilizarán dos tipos de topics; Actuación y Configuración.
 * Cada topic de actuación lleva ligado uno de configuración y puede haber
 * tantos como se precise.
 *  ACTUACION:
 *              A01/Material/leds
 *  CONFIGURACION
 *              A01/d:00124bXXXXXX/Conf
 *  PUBLICACION
 *              A01/client_id/status/fmt/json
 *
 * We also need space for the null termination
 */
#define BUFFER_SIZE 64
static char client_id[BUFFER_SIZE];
static char pub_topic[BUFFER_SIZE];
static char alarm_topic[BUFFER_SIZE];
static char sub_topic_Act[BUFFER_SIZE];
static char sub_topic_ConfA[BUFFER_SIZE];
static char sub_topic_ConfP[BUFFER_SIZE];

static uint8_t alarmOn;
/*---------------------------------------------------------------------------*/
/*
 * The main MQTT buffers.
 * We will need to increase if we start publishing more data.
 */
#define APP_BUFFER_SIZE 512
static struct mqtt_connection conn;
static char app_buffer[APP_BUFFER_SIZE];
/*---------------------------------------------------------------------------*/
static struct mqtt_message *msg_ptr = 0;
static struct etimer publish_periodic_timer;
static struct ctimer ct;
static char *buf_ptr;
static uint16_t seq_nr_value = 0;
/*---------------------------------------------------------------------------*/
static uip_ip6addr_t def_route;
/*---------------------------------------------------------------------------*/
/* Parent RSSI functionality */
extern int def_rt_rssi;
/*---------------------------------------------------------------------------*/
/* 
 * Lista con todos los valores de lecturas de los sensores. Para este caso solo
 * se tiene el valor de temperatura y voltaje de la batería.
*/
static alstom_mqtt_iot_sensor_reading_t *reading;
/*---------------------------------------------------------------------------*/
mqtt_client_config_t *conf;
/*---------------------------------------------------------------------------*/
/* 
 * Bandera que indica el topic al que debe subscribirse el dispositivo. Es 
 * incrementado por el evento de subscripción del cliente mqtt (mqtt_event) y 
 * reiniciada cuando se ha de actualizar la configuración (update_conf).
*/
int flag_sub_topic = 0;
/*---------------------------------------------------------------------------*/
PROCESS(mqtt_client_process, "CC26XX MQTT Client");
/*---------------------------------------------------------------------------*/
static void
publish_led_off(void *d)                                                                  //APAGA EL LED
{
  leds_off(ALSTOM_MQTT_IOT_STATUS_LED);
}
/*---------------------------------------------------------------------------*/
/*
* Handler de la subcripción a Actuaciónes. Será disparado cuando se envíe un mensaje
* binario (1/0) a un topic de actuación al que el dispositivo esté ligado. En este
* handler se comprueba que el topic es el correcto y en caso afirmativo se enciende
* el LED rojo. 
* Este handler gestionará todas las actuaciones por muchas operaciones a las que se 
* ligue el dispositivo. Es muy importante tener en cuenta las longitudes de los topics
* ya que si se falla en ese punto el dipositivo no responderá a las ordenes externas.
*/
static void
pub_handler_Act(const char *topic, uint16_t topic_len, const uint8_t *chunk,
            uint16_t chunk_len)
{
  DBG("Pub Handler: topic='%s' (len=%u), chunk_len=%u\n", topic, topic_len,chunk_len);

  /* Se comprueba la longitud del topic y del mensaje para comprobar si coinciden
  con los esperados ya que siempre la longitud es fija*/
  /* If we don't like the length, ignore */
  if((chunk_len != 1)||(topic_len!=14)) {
    	printf("Incorrect topic or chunk len. Ignored\n");
    	return;
  }

  /* Si el topic no coincide con la operacion asignada al Sensortag
  se ignora. Es muy importante ya que en caso contrario se activaría en todas
  las operaciones*/ 
  if(strncmp(&topic[4], conf->material, 5) == 0) {				//Si las cadenas no son iguales
    printf("Corresponde con el topic %s\n", conf->material);   
  /*
  * En caso de que el dispositivo estuviera suscrito a más de una operación este
  * es el lugar dónde se deben colocar los elseif de comprobación, al igual que el
  * if anterior.
  */
  }
  else{
    printf("Incorrect format\n");
    return;
  }
  /*Comprueba que el topic termina en leds, es una medida evitable, pero importante si 
  se introducen más topics por operación */
  if(strncmp(&topic[topic_len - 4], "leds", 4) == 0) {
    if(chunk[0] == '1') {
      leds_on(LEDS_RED);
    } else if(chunk[0] == '0') {
      leds_off(LEDS_RED);
    }
    return;
  }
  if(strncmp(&topic[topic_len - 4], "buzz", 4) == 0) {
    if(chunk[0] == '1') {
      buzzer_start(1000);
    } else if(chunk[0] == '0') {
      buzzer_stop();
    }
    return;
  }
}
/*---------------------------------------------------------------------------*/
/*
* Handler de la subcripción a Configuración. Habrá tantas subscripciones a configuraciones
* como a actuaciones, ya que cada actuación debe poder ser modificada dinámicamente. Cada topic
* de configuración lleva asigado un número por lo que es importante tener un registro externo 
* de las configuracion del dispositivo para poder cambiar una antigua por otra nueva sin fallos. 
*/
static void
pub_handler_ConfA(const char *topic, uint16_t topic_len, const uint8_t *chunk,
            uint16_t chunk_len)
{
  int i;
  DBG("Pub Handler: topic='%s' (len=%u), chunk_len=%u\n", topic, topic_len,chunk_len);

    /* Se comprueba la longitud del topic y del mensaje para comprobar si coninciden
    con los esperados
    If we don't like the length, ignore*/ 
  if((chunk_len != 5)||(topic_len!=23)) {
    	printf("Incorrect topic or chunk len. Ignored\n");
    	return;
  }

  /* Se comprueba si el mensaje iba para este Sensortag comprobando que coinciden los client_ID*/ 
  if(strncmp(&topic[4], client_id, 14) != 0) {				//Si las cadenas no son iguales
    printf("Incorrect format\n");
    return;
  }
  /*SEGURIDAD:se comprueba que el topic termina en Conf. Permite añadir topics en el mismo nivel*/
  if(strncmp(&topic[topic_len - 4],"Conf",4) == 0){
  	   for(i=0;i<chunk_len;i++){
  		    conf->material[i] = chunk[i];
  	   }
  	   state = MQTT_CLIENT_STATE_NEWCONFIG;
       mqtt_disconnect(&conn);
  	   return;
  }
}
/*---------------------------------------------------------------------------*/
/*
* Modifica los sensores que son publicados en el mensaje. Para ello recorre la lista de sensores añadidos 
* y modifaca la propiedad publish del que coincida con el nombre del topic. No es necesario resetear la conexión
* ya que por defecto esta propiedad se comprueba cada segundo.
*/
static void
pub_handler_ConfP(const char *topic, uint16_t topic_len, const uint8_t *chunk,
            uint16_t chunk_len)
{

  DBG("Pub Handler: topic='%s' (len=%u), chunk_len=%u\n", topic, topic_len,chunk_len);

    /* Se comprueba la longitud del topic y del mensaje para comprobar si coninciden
    con los esperados
    If we don't like the length, ignore*/ 
  if(topic_len < 26) {
      printf("Incorrect topic or chunk len. Ignored\n");
      return;
  }

  /* Se comprueba si el mensaje iba para este Sensortag comprobando que coinciden los client_ID*/ 
  if(strncmp(&topic[4], client_id, 14) != 0) {        //Si las cadenas no son iguales
    printf("Incorrect format\n");
    return;
  }
  /*SEGURIDAD:se comprueba que el topic termina en Conf. Permite añadir topics en el mismo nivel*/
  if(strncmp(&topic[topic_len - 3],"Pub",3) == 0){
     for(reading = alstom_mqtt_iot_sensor_first();reading != NULL; reading = reading->next) {
        if(strncmp(&topic[topic_len - 4 - strlen(reading->form_field)],reading->form_field,strlen(reading->form_field) ) == 0){
          DBG("Match en sensor\n");
          if(chunk[0] == '0') {
            reading->publish = 0;
            snprintf(reading->converted, ALSTOM_MQTT_IOT_CONVERTED_LEN, "\"N/A\"");
            DBG("Desactivada publicación de %s\n",reading->form_field);
            return;
          } else if(chunk[0] == '1') {
            reading->publish = 1;
            init_sensor_readings();
            DBG("Activada publicación de %s\n",reading->form_field);
            return;
          }
        }   
      }
      return;
  }
  if(strncmp(&topic[topic_len - 3],"Int",3) == 0){
     for(reading = alstom_mqtt_iot_sensor_first();reading != NULL; reading = reading->next) {
        if(strncmp(&topic[topic_len - 4 - strlen(reading->form_field)],reading->form_field,strlen(reading->form_field) ) == 0){
          reading->interval = atoi(chunk);
          DBG("Modificado Intervalo de %s. Nuevo intervalo: %d\n",reading->form_field,reading->interval);
          return;
        }   
      }
      return;
  }
}
/*---------------------------------------------------------------------------*/
/*
* Es el handler que se ejecuta con las interrupciones de conexión del cliente (mqtt_register).
* Gestiona el correcto funiconamiento del cliente y permite introducir acciones ante
* ciertos eventos como la subscripción a un topic, publicación o desconexión.
*/
static void
mqtt_event(struct mqtt_connection *m, mqtt_event_t event, void *data)
{
  switch(event) {
  /*Conexion cliente/Broker realizada*/
  case MQTT_EVENT_CONNECTED: {
    DBG("APP - Application has a MQTT connection\n");
    timer_set(&connection_life, CONNECTION_STABLE_TIME);
    state = MQTT_CLIENT_STATE_CONNECTED;
    break;
  }
  /*Cliente desconectado*/
  case MQTT_EVENT_DISCONNECTED: {
    DBG("APP - MQTT Disconnect. Reason %u\n", *((mqtt_event_t *)data));

    /* Do nothing if the disconnect was the result of an incoming config */
    if(state != MQTT_CLIENT_STATE_NEWCONFIG) {
      state = MQTT_CLIENT_STATE_DISCONNECTED;
      process_poll(&mqtt_client_process);
    }
    /*
    else{
      process_poll(&mqtt_client_process);
    }*/
    break;
  }
  /*RECIBE una nueva publicacion de algún tema subscrito*/
  case MQTT_EVENT_PUBLISH: {
    msg_ptr = data;

    /* Implement first_flag in publish message? */
    if(msg_ptr->first_chunk) {
      msg_ptr->first_chunk = 0;
      DBG("APP - Application received a publish on topic '%s'. Payload "
          "size is %i bytes. Content:\n\n",
          msg_ptr->topic, msg_ptr->payload_length);
    }
    if(strlen(msg_ptr->topic) == 14){
    	pub_handler_Act(msg_ptr->topic, strlen(msg_ptr->topic), msg_ptr->payload_chunk,
                msg_ptr->payload_length);
    }
    if(strlen(msg_ptr->topic) == 23){
    	pub_handler_ConfA(msg_ptr->topic, strlen(msg_ptr->topic), msg_ptr->payload_chunk,
                msg_ptr->payload_length);
    }
    if(strlen(msg_ptr->topic) > 23){
      pub_handler_ConfP(msg_ptr->topic, strlen(msg_ptr->topic), msg_ptr->payload_chunk,
                msg_ptr->payload_length);
    }
    break;
  }
  /*Subscripcion a topic realizada con éxito*/
  case MQTT_EVENT_SUBACK: {
    DBG("APP - Application is subscribed to topic successfully\n");
    /*
    * Cada vez que se realiza una subscripción con éxito esta bandera aumenta en 1.
    * Esto permite realizar la siguiente subscripción. Así hasta que la bandera supere
    * el número máximo de subscripciones.
    */
    flag_sub_topic = flag_sub_topic + 1;
    break;
  }
  /*Des-subscripcion realizada con éxito*/
  case MQTT_EVENT_UNSUBACK: {
    DBG("APP - Application is unsubscribed to topic successfully\n");
    break;
  }
  /*Publicacion completada*/
  case MQTT_EVENT_PUBACK: {
    DBG("APP - Publishing complete.\n");
    break;
  }
  default:
    DBG("APP - Application got a unhandled MQTT event: %i\n", event);
    break;
  }
}
/*---------------------------------------------------------------------------*/
/*
* Construye el topic de publicación a partir de los datos del dispostivo
* y el formato impuesto. Este topic debe estar reservado al comienzo del
* código en este archivo.
*/
static int
construct_pub_topic(void)
{
  int len = snprintf(pub_topic, BUFFER_SIZE, "%s/%s/%s/fmt/json",
                     conf->sala,client_id,conf->event_type_id);

  /* len < 0: Error. Len >= BUFFER_SIZE: Buffer too small */
  if(len < 0 || len >= BUFFER_SIZE) {
    printf("Pub Topic: %d, Buffer %d\n", len, BUFFER_SIZE);
    return 0;
  }
  int lena = snprintf(alarm_topic, BUFFER_SIZE, "%s/%s/alarm/SenOff",
                     conf->sala,client_id);

  /* len < 0: Error. Len >= BUFFER_SIZE: Buffer too small */
  if(lena < 0 || lena >= BUFFER_SIZE) {
    printf("Pub Topic: %d, Buffer %d\n", len, BUFFER_SIZE);
    return 0;
  }
  return 1;
}
/*---------------------------------------------------------------------------*/
/*
* Construye los topics de subscripción a partir de los datos del dispositivo
* y el formato impuesto. Crea varios de ellos. Es necesario que estos topics
* hayan sido reservados al comienzo del código en este archivo. 
* Los topics de actuación son creados explicitamente en el numero necesario. Podría
* haberse utilizado el comando "+" pero en ese caso actuaría ante cada orden que 
* fuera a cada Sensortag, con el consecuente gasto de recursos y batería.
* En cambio los topics de configuracion son reservados con el comando "+" ya que 
* ya vienen filtrados por el Id del cliente. 
*/
static int
construct_sub_topic(void)
{
  /*Construción topics de Actuación*/
  int len = snprintf(sub_topic_Act, BUFFER_SIZE, "%s/%s/%s",conf->sala,conf->material,conf->cmd_type);
  DBG("Creado topic: %s/%s/%s\n",conf->sala,conf->material,conf->cmd_type);
  /* len < 0: Error. Len >= BUFFER_SIZE: Buffer too small */
  if(len < 0 || len >= BUFFER_SIZE) {
    printf("Sub Topic: %d, Buffer %d\n", len, BUFFER_SIZE);
    return 0;
  }
  /*Construcción topics de configuración Actuación*/
  int lenca = snprintf(sub_topic_ConfA,BUFFER_SIZE,"%s/%s/Conf",conf->sala,client_id);
  DBG("Creado topic: %s/%s/Conf\n",conf->sala,client_id);
  if(lenca < 0 || lenca >= BUFFER_SIZE) {
    printf("Sub Topic: %d, Buffer %d\n", lenca, BUFFER_SIZE);
    return 0;
  }
  /*Construcción topics de configuración Publicación*/
  int lencp = snprintf(sub_topic_ConfP,BUFFER_SIZE,"%s/%s/%s/%s",conf->sala,client_id,conf->cmd_type,conf->cmd_type);
  DBG("Creado topic: %s/%s/%s/%s\n",conf->sala,client_id,conf->cmd_type,conf->cmd_type);
  if(lencp < 0 || lencp >= BUFFER_SIZE) {
    printf("Sub Topic: %d, Buffer %d\n", lencp, BUFFER_SIZE);
    return 0;
  }
  return 1;
}
/*---------------------------------------------------------------------------*/
/*
* CONSTRUYE EL CLIENT_ID A PARTIR DE LA DIRECCION MAC (DEVICE_ID) SEGÚN EL FORMATO
* d:<Device_ID>, SIMPLIFICADO DE d:<ORG_ID>:<Type_ID>:<Device_ID>
*/
static int
construct_client_id(void)                                                                 
{
  int len = snprintf(client_id, BUFFER_SIZE, "d:%02x%02x%02x%02x%02x%02x",
                     linkaddr_node_addr.u8[0], linkaddr_node_addr.u8[1],
                     linkaddr_node_addr.u8[2], linkaddr_node_addr.u8[5],
                     linkaddr_node_addr.u8[6], linkaddr_node_addr.u8[7]);

  /* len < 0: Error. Len >= BUFFER_SIZE: Buffer too small */
  if(len < 0 || len >= BUFFER_SIZE) {
    printf("Client ID: %d, Buffer %d\n", len, BUFFER_SIZE);
    return 0;
  }

  return 1;
}
/*---------------------------------------------------------------------------*/
/* 
* ACTUALIZA LA INFORMACION SOBRE LOS TOPIC SUBSCRITOS Y DE PUBLICACIÓN,
* ESTA FUNCIÓN ES ACTIVADA DESDE LA MÁQUINA DE ESTADOS, NO SE DEBE REALIZAR 
* UNA LLAMADA DIRECTA.
*/
static void
update_config(void)
{
  /*Construcción del Client_ID, en caso de que la linkaddr haya sido modificada*/
  if(construct_client_id() == 0) {
    /* Fatal error. Client ID larger than the buffer */
    state = MQTT_CLIENT_STATE_CONFIG_ERROR;
    return;
  }
  /*Modificado de los topics de subscripcion, estos pueden ser modificados por
  mensajes recibidos de ellos mismos, e.g Código de operación para topic*/
  if(construct_sub_topic() == 0) {
    /* Fatal error. Topic larger than the buffer */
    state = MQTT_CLIENT_STATE_CONFIG_ERROR;
    return;
  }
  /*Construcción de los topics de publicación*/
  if(construct_pub_topic() == 0) {
    /* Fatal error. Topic larger than the buffer */
    state = MQTT_CLIENT_STATE_CONFIG_ERROR;
    return;
  }

  /* Reset the counter and the subscription flag */
  seq_nr_value = 0;
  flag_sub_topic = 0;
  /*Resetea el cliente, imponiendo un estado de inicio de nuevo*/
  state = MQTT_CLIENT_STATE_INIT;

  /*
   * Schedule next timer event ASAP
   *
   * If we entered an error state then we won't do anything when it fires.
   *
   * Since the error at this stage is a config error, we will only exit this
   * error state if we get a new config.
   */
  etimer_set(&publish_periodic_timer, 0);

  return;
}
/*---------------------------------------------------------------------------*/
/*
* INICIA CONFIGURACIÓN, COPIA TODAS LAS CONSTANTES EN LA ESTRUCUTURA CONF
* LAS LONGITUDES TIENEN QUE SER AJUSTADAS SI SE MODIFICA ALGUNO DE LOS PARÁMETROS.
*/
static int
init_config()
{
  /* Populate configuration with default values */
  memset(conf, 0, sizeof(mqtt_client_config_t));
  /*Eliminado la copia de Org_ID y Type_ID ya que client_ID no se forma con ellos*/
  memcpy(conf->event_type_id, ALSTOM_MQTT_IOT_DEFAULT_EVENT_TYPE_ID, 7);
  memcpy(conf->broker_ip, broker_ip, strlen(broker_ip));
  memcpy(conf->cmd_type, ALSTOM_MQTT_IOT_DEFAULT_SUBSCRIBE_CMD_TYPE, 1);
  memcpy(conf->user_id,ALSTOM_MQTT_IOT_DEFAULT_USERNAME_ID,24);
  memcpy(conf->auth_token, ALSTOM_MQTT_IOT_DEFAULT_AUTH_TOKEN, 20);
  memcpy(conf->material,ALSTOM_MQTT_IOT_DEFAULT_MATERIAL,5);
  memcpy(conf->sala,ALSTOM_MQTT_IOT_DEFAULT_SALA,3);
  conf->broker_port = ALSTOM_MQTT_IOT_DEFAULT_BROKER_PORT;
  conf->pub_interval = ALSTOM_MQTT_IOT_DEFAULT_PUBLISH_INTERVAL;

  return 1;
}

/*---------------------------------------------------------------------------*/
/*
* Realiza las subcripciones a los topics de actuación y configuracion. Las
* subscripciones deben ser llevadas a cabo una tras la finalización de la 
* anterior por lo que la bandera permite la subscripción en el momento
* adecuado evitando el llenado de la cola (la no subscripcion).
* IMPORTANTE: NIVEL DE QoS == 1 
*/
static void
subscribe()
{
  /* Publish MQTT topic in IBM quickstart format */
  mqtt_status_t status;
  switch (flag_sub_topic){
    case 0:{
      DBG("APP - Subscribing!\n");
      status = mqtt_subscribe(&conn, NULL, sub_topic_Act, MQTT_QOS_LEVEL_1);
      if(status == MQTT_STATUS_OUT_QUEUE_FULL) {
        DBG("APP - Tried to subscribe but command queue was full!\n");
      }
      break;
    }
    case 1:{
      DBG("APP - Subscribing!\n");
      status = mqtt_subscribe(&conn, NULL, sub_topic_ConfA, MQTT_QOS_LEVEL_1);
      if(status == MQTT_STATUS_OUT_QUEUE_FULL) {
        DBG("APP - Tried to subscribe but command queue was full!\n");
      }   
      break;
    }
    case 2:{
      DBG("APP - Subscribing!\n");
      status = mqtt_subscribe(&conn, NULL, sub_topic_ConfP, MQTT_QOS_LEVEL_1);
      if(status == MQTT_STATUS_OUT_QUEUE_FULL) {
        DBG("APP - Tried to subscribe but command queue was full!\n");
      }   
      break;
    }
    default:
      break;
  }
}
/*---------------------------------------------------------------------------*/
/*
* Publica en el broker según el topic de publicación. En este caso solo publica los datos
* refrentes al dispositivo y los datos de temperatura y voltaje de la batería. Unicos
* valores de la lista reading.
*/
static void
publish(void)
{
  /* Publish MQTT topic in IBM quickstart format */
  int len;
  int remaining = APP_BUFFER_SIZE;
  char def_rt_str[64];

  seq_nr_value++;

  buf_ptr = app_buffer;

  len = snprintf(buf_ptr, remaining,
                 "{"
                 "\"d\":{"
                 "\"myName\":\"%s\","
                 "\"Seq #\":%d,"
                 "\"Uptime (sec)\":%lu",
                 BOARD_STRING, seq_nr_value, clock_seconds());                                      //GUARDA EN BUFFER LOS DATOS DEL SENSORTAG

  if(len < 0 || len >= remaining) {
    printf("Buffer too short. Have %d, need %d + \\0\n", remaining, len);
    return;
  }

  remaining -= len;
  buf_ptr += len;

  /* Put our Default route's string representation in a buffer */
  memset(def_rt_str, 0, sizeof(def_rt_str));
  alstom_mqtt_iot_ipaddr_sprintf(def_rt_str, sizeof(def_rt_str),
                                 uip_ds6_defrt_choose());

  len = snprintf(buf_ptr, remaining, ",\"Def Route\":\"%s\",\"RSSI (dBm)\":%d",                     //DIRECCION Y RSSI
                 def_rt_str, def_rt_rssi);

  if(len < 0 || len >= remaining) {
    printf("Buffer too short. Have %d, need %d + \\0\n", remaining, len);
    return;
  }
  remaining -= len;
  buf_ptr += len;

  memcpy(&def_route, uip_ds6_defrt_choose(), sizeof(uip_ip6addr_t));

  for(reading = alstom_mqtt_iot_sensor_first();reading != NULL; reading = reading->next) {
    if(reading->publish && reading->raw != CC26XX_SENSOR_READING_ERROR) {
      /*
      * Dado que hay diferentes frecuencias de publicación, solo se publica un valor si ha cambiado con respecto
      * al dato anterior. Evita sobrecarga en la publicación y repetición de medidas ya enviadas.
      */
      if(reading->changed == 1){
        len = snprintf(buf_ptr, remaining,",\"%s (%s)\":%s", reading->descr, reading->units,reading->converted);                                                           //LECTURAS DE SENSORES
        if(len < 0 || len >= remaining) {
          printf("Buffer too short. Have %d, need %d + \\0\n", remaining, len);
          return;
        }
        remaining -= len;
        buf_ptr += len;
      }
    }
  }

  len = snprintf(buf_ptr, remaining, "}}");

  if(len < 0 || len >= remaining) {
    printf("Buffer too short. Have %d, need %d + \\0\n", remaining, len);
    return;
  }

  mqtt_publish(&conn, NULL, pub_topic, (uint8_t *)app_buffer,
               strlen(app_buffer), MQTT_QOS_LEVEL_0, MQTT_RETAIN_OFF);                                //PUBLICA
                                                                                                      //IMP:SIN RETENCION Y POR LO TANTO QoS==0
  DBG("APP - Publish!\n");
}
/*---------------------------------------------------------------------------*/
/*
* Publica en el broker según el topic de publicación. En este caso solo publica los datos
* refrentes al dispositivo y los datos de temperatura y voltaje de la batería. Unicos
* valores de la lista reading.
*/
static void
alarm(void)
{
  int len;
  int remaining = APP_BUFFER_SIZE;
  char def_rt_str[64];

  seq_nr_value++;

  buf_ptr = app_buffer;

  len = snprintf(buf_ptr, remaining,
                 "{"
                 "\"d\":{"
                 "\"myName\":\"%s\","
                 "\"Seq #\":%d,"
                 "\"Uptime (sec)\":%lu",
                 BOARD_STRING, seq_nr_value, clock_seconds());                                      //GUARDA EN BUFFER LOS DATOS DEL SENSORTAG

  if(len < 0 || len >= remaining) {
    printf("Buffer too short. Have %d, need %d + \\0\n", remaining, len);
    return;
  }

  remaining -= len;
  buf_ptr += len;

  /* Put our Default route's string representation in a buffer */
  memset(def_rt_str, 0, sizeof(def_rt_str));
  alstom_mqtt_iot_ipaddr_sprintf(def_rt_str, sizeof(def_rt_str),
                                 uip_ds6_defrt_choose());

  len = snprintf(buf_ptr, remaining, ",\"Def Route\":\"%s\",\"RSSI (dBm)\":%d",                     //DIRECCION Y RSSI
                 def_rt_str, def_rt_rssi);

  if(len < 0 || len >= remaining) {
    printf("Buffer too short. Have %d, need %d + \\0\n", remaining, len);
    return;
  }
  remaining -= len;
  buf_ptr += len;

  memcpy(&def_route, uip_ds6_defrt_choose(), sizeof(uip_ip6addr_t));

  len = snprintf(buf_ptr, remaining, ",\"Alarma\":\"OFF");
  if(len < 0 || len >= remaining) {
    printf("Buffer too short. Have %d, need %d + \\0\n", remaining, len);
    return;
  }
  remaining -= len;
  buf_ptr += len;
  len = snprintf(buf_ptr, remaining, "}}");

  if(len < 0 || len >= remaining) {
    printf("Buffer too short. Have %d, need %d + \\0\n", remaining, len);
    return;
  }

  mqtt_publish(&conn, NULL, alarm_topic, (uint8_t *)app_buffer,
               strlen(app_buffer), MQTT_QOS_LEVEL_0, MQTT_RETAIN_OFF);                                //PUBLICA
                                                                                                      //IMP:SIN RETENCION Y POR LO TANTO QoS==0
  DBG("APP - Publish!\n");
}
/*---------------------------------------------------------------------------*/
/*
* SE CONECTA AL BROKER, NECESARIO LA DIRECCIÓN IP Y EL PUERTO
*/
static void
connect_to_broker(void)                                                                   
{
  /* Connect to MQTT server */
  mqtt_connect(&conn, conf->broker_ip, conf->broker_port,
               conf->pub_interval * 3);

  state = MQTT_CLIENT_STATE_CONNECTING;
}
/*---------------------------------------------------------------------------*/
/*
* Maquina de estados que gestiona las conexiones, subscripciones y pubilcaciones del
* cliente. Es disparada mediante el proceso pricncipal.
*/
static void
state_machine(void)
{
  switch(state) {
  /*
  * Inicio del cliente MQTT, registra la conexión (cliente) y fija el nombre de usuario y contraseña 
  */  
  case MQTT_CLIENT_STATE_INIT:
    /* If we have just been configured register MQTT connection */
    mqtt_register(&conn, &mqtt_client_process, client_id, mqtt_event,
                  MQTT_CLIENT_MAX_SEGMENT_SIZE);

      if(strlen(conf->auth_token) == 0) {                                                 
        printf("User name set, but empty auth token\n");
        state = MQTT_CLIENT_STATE_ERROR;
        break;
      } else {                                                                            //SI AUTH_TOKEN != 0 
        mqtt_set_username_password(&conn, conf->user_id,                                  //FIJA EL USUARIO DE LA CONEXIÓN
                                   conf->auth_token);                                     //FIJA LA CONTRASEÑA DE LA CONEXIÓN
      }
    

    /* _register() will set auto_reconnect. We don't want that. */
    conn.auto_reconnect = 0;
    connect_attempt = 1;

    /*
     * Wipe out the default route so we'll republish it every time we switch to
     * a new broker
     */
    memset(&def_route, 0, sizeof(def_route));

    state = MQTT_CLIENT_STATE_REGISTERED;
    DBG("Init\n");
    /* Continue */
  /*
  * Conexión al Broker tras registro del cliente, el paso a CONNECTING lo hace la función connect_to_broker
  */
  case MQTT_CLIENT_STATE_REGISTERED:
    if(uip_ds6_get_global(ADDR_PREFERRED) != NULL) {
      /* Registered and with a public IP. Connect */
      DBG("Registered. Connect attempt %u\n", connect_attempt);
      connect_to_broker();
    }
    etimer_set(&publish_periodic_timer, ALSTOM_MQTT_IOT_NET_CONNECT_PERIODIC);
    return;
    break;
  /*Estableciendo conexión, enciende el led verde*/
  case MQTT_CLIENT_STATE_CONNECTING:
    leds_on(ALSTOM_MQTT_IOT_STATUS_LED);
    ctimer_set(&ct, CONNECTING_LED_DURATION, publish_led_off, NULL);
    /* Not connected yet. Wait */
    DBG("Connecting (%u)\n", connect_attempt);
    break;
  /*
  *Cliente conectado a Broker con éxito, es disparado por mqtt_event
  */
  case MQTT_CLIENT_STATE_CONNECTED:
    /*
    * MUY IMPORTANTE: No hay break por lo que aunque el estado es CONNECTED,
    * se ejecuta código de PUBLISHING
    */
    /* Continue */
  case MQTT_CLIENT_STATE_PUBLISHING:
    /* If the timer expired, the connection is stable. */
    if(timer_expired(&connection_life)) {
      /*
       * Intentionally using 0 here instead of 1: We want RECONNECT_ATTEMPTS
       * attempts if we disconnect after a successful connect
       */
      connect_attempt = 0;
    }

    if(mqtt_ready(&conn) && conn.out_buffer_sent) {                           //Conexion lista
      /* Connected. Publish */
      if(state == MQTT_CLIENT_STATE_CONNECTED) {                              //Si el estado es CONNECTED
        /*
        * Dependiendo el valor de la bandera nos permite subscribirnos 
        * a diferentes topics,para acelerar el proceso se fuerza la 
        * ejecucion recursiva de state_machine.
        */
        if(flag_sub_topic<3){
          subscribe();
        }else{
          state = MQTT_CLIENT_STATE_PUBLISHING;
        }    
        process_poll(&mqtt_client_process);
      } else {
        /*
        * Si la máquina de estados ha saltado por acción del operario, se publica la alarma 
        * y se baja la bandera. En caso contario se realiza una publicación de los sensores.
        */
        if(alarmOn == 1){
          alarm(); 
          alarmOn = 0; 
        }else{
        leds_on(ALSTOM_MQTT_IOT_STATUS_LED);                                  //Enciende LED verde
        ctimer_set(&ct, PUBLISH_LED_ON_DURATION, publish_led_off, NULL);
        publish();
        /*
        * Tras la publicación, se chequea la lista de sensores y se modifica el valor changed. Así solo
        * se publicarán aquellos valores que hayan sido modificados.
        */
        for(reading = alstom_mqtt_iot_sensor_first();reading != NULL; reading = reading->next) {
          compare_and_update(reading);
        }                                                            
      }
      /*
      * Se comparan las frecuencias de los diferentes sensores, aquel que esté activo
      * y cuyo intervalo de toma de datos sea menor, será el intervalo de publicación 
      * del sensortag.
      */
      uint32_t minterval = ALSTOM_MQTT_IOT_DEFAULT_PUBLISH_INTERVAL/CLOCK_SECOND;

      for(reading = alstom_mqtt_iot_sensor_first();reading != NULL; reading = reading->next) {
        if((reading->interval < minterval) && (reading->publish == 1)){
          minterval=reading->interval;
        }
      }
      etimer_set(&publish_periodic_timer, minterval*CLOCK_SECOND);

      DBG("Publishing\n");
      /* Return here so we don't end up rescheduling the timer */
      return;
    } else {
      /*
       * Our publish timer fired, but some MQTT packet is already in flight
       * (either not sent at all, or sent but not fully ACKd).
       *
       * This can mean that we have lost connectivity to our broker or that
       * simply there is some network delay. In both cases, we refuse to
       * trigger a new message and we wait for TCP to either ACK the entire
       * packet after retries, or to timeout and notify us.
       */
      DBG("Publishing... (MQTT state=%d, q=%u)\n", conn.state,conn.out_queue_full);
    }
    break;
  /*Estado de desconexión*/
  case MQTT_CLIENT_STATE_DISCONNECTED:
    DBG("Disconnected\n");
    /*Si no se han superado los intentos de conexión máximos (o son infinitos)*/
    if(connect_attempt < RECONNECT_ATTEMPTS ||RECONNECT_ATTEMPTS == RETRY_FOREVER) {
      /* Disconnect and backoff */
      clock_time_t interval;
      mqtt_disconnect(&conn);   //Desconecta
      connect_attempt++;        //Suma un intento de conexion 

      /*Aumenta el intervalo de reconexion dependiendo el numero de intentos realizados*/
      interval = connect_attempt < 3 ? RECONNECT_INTERVAL << connect_attempt :RECONNECT_INTERVAL << 3;

      DBG("Disconnected. Attempt %u in %lu ticks\n", connect_attempt, interval);

      etimer_set(&publish_periodic_timer, interval);

      state = MQTT_CLIENT_STATE_REGISTERED;
      return;
    } else {
      /* Max reconnect attempts reached. Enter error state */
      state = MQTT_CLIENT_STATE_ERROR;
      DBG("Aborting connection after %u attempts\n", connect_attempt - 1);
    }
    break;
  /*Modifica la configración actualizandola y reiniciando la conexion del cliente con el broker*/
  case MQTT_CLIENT_STATE_NEWCONFIG:
    /* Only update config after we have disconnected */
    if(conn.state == MQTT_CONN_STATE_NOT_CONNECTED) {
      update_config();
      DBG("New config\n");

      /* update_config() scheduled next pass. Return */
      return;
    }
    break;
  case MQTT_CLIENT_STATE_CONFIG_ERROR:
    /* Idle away. The only way out is a new config */
    printf("Bad configuration.\n");
    return;
  case MQTT_CLIENT_STATE_ERROR:
  default:
    leds_on(ALSTOM_MQTT_IOT_STATUS_LED);
    /*
     * 'default' should never happen.
     *
     * If we enter here it's because of some error. Stop timers. The only thing
     * that can bring us out is a new config event
     */
    printf("Default case: State=0x%02x\n", state);
    return;
  }

  /* If we didn't return so far, reschedule ourselves */
  etimer_set(&publish_periodic_timer, STATE_MACHINE_PERIODIC);
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(mqtt_client_process, ev, data)
{

  PROCESS_BEGIN();

  printf("CC26XX MQTT Client Process\n");

  conf = &alstom_mqtt_iot_config.mqtt_config;          //Carga la configuración 
  if(init_config() != 1) {                             //Ejecuta init_config, cargando la estructura de configuración del cliente
    PROCESS_EXIT();
  }

  update_config();                                    //Actualiza, cargando el cliente_ID y los topics de subscripcion y publicacion 

  /* Main loop */
  while(1) {

    PROCESS_YIELD();
    /*Si llega evento de sensores (disparo de publicación)*/
    if(ev == sensors_event && data == ALSTOM_MQTT_IOT_MQTT_PUBLISH_TRIGGER) {
      if(state == MQTT_CLIENT_STATE_ERROR) {
        connect_attempt = 1;
        state = MQTT_CLIENT_STATE_REGISTERED;
      }
    }
    /*Si salta el timer de publicación periodica, evento de pooling (disparado por desconexión),
     publicacion o evento de sensores*/
    if((ev == PROCESS_EVENT_TIMER && data == &publish_periodic_timer) ||
       ev == PROCESS_EVENT_POLL ||
       ev == alstom_mqtt_iot_publish_event ||
       (ev == sensors_event && data == ALSTOM_MQTT_IOT_MQTT_PUBLISH_TRIGGER)) {
      state_machine();
    }
    /*Si salta evento de carga de configuracion por defecto*/
    if(ev == alstom_mqtt_iot_load_config_defaults) {
      init_config();
      etimer_set(&publish_periodic_timer, NEW_CONFIG_WAIT_INTERVAL);
    }
    /*
    * Si el operario pulsa el botón se apaga el led del sensortag y
    * se envía un mensaje de alarma al broker.
    */
    if(ev == sensors_event && data == ALSTOM_MQTT_IOT_OP_LED_OFF){
      alarmOn = 1;
      leds_off(ALSTOM_MQTT_IOT_OP_LED);
      state_machine();
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
/**
 * @}
 */
