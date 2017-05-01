ASLTOM MQTT IoT Readme
======================
OBJETIVOS
----------
El objetivo es obtener conectividad con el MQTT Broker de ALSTOM, eliminando  toda funcionalidad superficial para conseguir el mejor rendimiento posible.  Además se persigue añadir un modo de bajo consumo, gracias al uso de QoS en nivel 1 (en principio) y modos de lpm para CC2650 disponibles junto a Contiki. Por último, se pretende configurar la encriptación AES-128 como medida básica de seguridad entre los nodos de la red y el gateway (Weptech saker).

Publicación
-----------
Este firmware se ha desarrollado a partir del ejemplo CC26xx Web Demo disponible en el repositorio de Contiki. En este ejemplo se obtienen todos los datos de los sensores del Sensortag y se publican en formato JSON en el siguiente topic:
`iot-2/evt/status/fmt/json`  
 En un principio se ha mantenido la funcionalidad porque puede ser interesante el uso de estos sensores en otras aplicaciones, aunque para trabajar como actuadores sería contraproducente al consumir recursos innecesarios. Estas publicaciones se realizan de forma automática en cada cierto intervalo de tiempo, fijado por la constante ALSTOM_MQTT_IOT_DEFAULT_PUBLISH_INTERVAL.  
 
 Subscripción  
 ------------
 Cada Sensoratag se subscribe a dos topics, uno destinado a operación y el otro destiando a configuración.
 ### Operación  
 El topics de operación sigue el siguiente esquema:
 `sala/operación/leds`  
 Las longitudes de cada palabra que forma el topic están regladas siendo en cada caso:  
 * sala -> 3 caracteres. Ej: A01  
 * operación -> 5 caracteres Ej: PIN01
 * leds -> 4 caracteres. Es fijo y no modificable.  
   
El total de caracteres de este topic sería 3+1+5+1+4 (contando los slashes) = 14. Por ejemplo: `A01/PIN01/leds`.  
El parámetro de sala tiene que ser fijado antes de compilar el código, mediante la constante ALSTOM_MQTT_IOT_DEFAULT_SALA. Pero el parámetro operación puede ser modificado mediante la constante ALSTOM_MQTT_IOT_DEFAULT_TIPO_OP (antes de compilar) o mediante el topic de configuración como se explicará a continuación.  
El uso de este topic es muy sencillo, solo es necesario enviar como payload un caracter binario (0 o 1). 1 para encender el led y 0 para apagarlo. También puede ser apagado el led de forma "local" pulsando el botón del Sensortag ms cercano al led.

### Configuración
El topic de configuración permite cambiar la subscripción del topic se operación para cada Sensortag. Es una funcionalidad interesante porque ofrece la posibilidad de modificar las subscripciones sin necesidad de reflashear el dispositivo. El topic de configuración sigue el siguiente esquema:  
`sala/client-id/Op_Mask`  
Al igual que para operación, las longitudes de cada parámetro están regladas:  
* sala -> 3 caracteres. Ej: A01
* client_id -> 14 caracteres. Sigue un esquema prefijado: `d:00124bABCDEFG` siendo fijos los caracteres d:00124b y variables ABCDEFG.Los carcteres fijos correponden a los 6 primeros dígitos hexadecimales de la dirección IEEE configurada en contiki-conf.h (dentro de la carpeta de la plataforma srf06-cc26xx) mediante la constante IEEE_ADDR_CONF_ADDRESS. Los caracteres variables corresponden a los 6 últimos dígitos de la dirección MAC del dispositivo (puede ser consultada en el programa Uniflash.
* Op_Mask -> 7 caracteres. Es fijo y no modificable.
El total de caracteres de este topic sería 3+1+14+1+7 (contando slashes) = 26. Por ejemplo: `A01/d:00124b30BCE8/Op_Mask`.  
Al igual que con el topics de operación el topic sala debe ser configurado antes de compilar el código. Este parámetro es compartido entre ambos topics (no tendría sentido de otra forma). Client-id viene fijado por el propio dispositivo y no es modificable, es un identificador único de cada Sensortag, y es lo que permite configurar cada uno de forma individual.

 

MQTT Client
----------------------------
The device will try to connect to the broker over NAT64, so you will
need a NAT64 gateway in your network to make this work. If this is not an option for you, you can
configure the device to publish to a local MQTT broker over end-to-end IPv6.
See below on how to change the destination broker's address.

For the SensorTag, changes to the MQTT configuration get saved in external
flash and persist across device restarts. The same does not hold true for
Srf+EM builds.

You can then use this to toggle LEDs. In order to public data using Mosquitto Broker, the terminal commands needed are the following:

`mosquitto_pub -h <broker IP> -m "1" -t sala/tipo_op/leds`  
`mosquitto_pub -h <broker IP> -m "PIN02" -t sala/client_ID/Op_Mask`

The message (-m) can change depending waht the user want to send.
Bear in mind that, even though the topic suggests that messages are of json
format, they are in fact not. This was done in order to avoid linking a json
parser into the firmware.
