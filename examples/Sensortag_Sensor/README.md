ALSTOM MQTT IoT SENSOR
======================
OBJETIVOS
----------
El objetivo es obtener conectividad con el MQTT Broker de ALSTOM, eliminando  toda funcionalidad superficial para conseguir el mejor rendimiento posible.  Además se persigue añadir un modo de bajo consumo, gracias al uso de QoS en nivel 1 (en principio) y modos de lpm para CC2650 disponibles junto a Contiki. Por último, se pretende configurar la encriptación AES-128 como medida básica de seguridad entre los nodos de la red y el gateway (Weptech saker).  
En este caso concreto la funcionalidad principal del Sensortag es la de funcionar como sensor, enviando al broker la información de sus sensores en formato JSON cada cierto intervalo de tiempo fijado por el usuario.

Publicación
-----------
Este firmware se ha desarrollado a partir del ejemplo CC26xx Web Demo disponible en el repositorio de Contiki. La función es publicar los datos obtenidos de los sensores en el siguiente topic MQTT en formato JSON para su tratamiento posterior:  
`sala/Sensortag/status/fmt/json`  
Estas publicaciones se realizan de forma automática en cada cierto intervalo de tiempo, fijado por la constante ALSTOM_MQTT_IOT_DEFAULT_PUBLISH_INTERVAL. Dado que la publicación de estos datos es la función principal de esta aplicación, este intervalo será relativamente bajo(del orden de segundos).Debido a limitaciones de la tecnología utilizada el mínimo intervalo de envío está fijado en 5 segundos por lo que este firmware no es adecuado para medir variaciones rápidas de sensores como el acelerómetro de 3 ejes.  
 
Subscripción  
-----------
 Cada Sensortag se subscribe a un topic destinado a la configuración del intervalo de publicación.
 
### Operación
Este topic permite cambiar el intervalo de publicación de los datos. Es útil ya que permite ser usado para diferentes situaciones o escenarios sin tener que pasar por la recarga del firmware. Sigue el siguiente esquema:  
'sala/client_id/Intervalo'  
Cada uno de los términos tiene una longitud reglada y no puede ser modificada:  
  
* Sala --> 3 caracteres. Ej: A01
* client_id --> 14 caracteres. Ej: d:000124bXXXXXX
* Intervalo --> 9 caracteres. Es fijo y no modificable.

La longitud total del topic es de *3+1+14+1+9=27* caracteres. Por ejemplo: `A01/d:000124b30BD87/Intervalo` 
 

MQTT Client & Mosquitto Broker
------------------------------
The device will try to connect to the broker over NAT64, so you will
need a NAT64 gateway in your network to make this work. If this is not an option for you, you can
configure the device to publish to a local MQTT broker over end-to-end IPv6.
See below on how to change the destination broker's address.

For the SensorTag, changes to the MQTT configuration get saved in external
flash and persist across device restarts. The same does not hold true for
Srf+EM builds.In order to public data using Mosquitto Broker, the terminal commands needed are the following:

`mosquitto_pub -h <broker IP> -m "20" -t sala/client_id/intervalo`  

The message (-m) can change depending waht the user want to send.
Bear in mind that, even though the topic suggests that messages are of json
format, they are in fact not. This was done in order to avoid linking a json
parser into the firmware.
