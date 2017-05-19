ALSTOM MQTT IoT ACTUADOR
======================
OBJETIVOS
----------
El objetivo de este firmware es obtener conectividad con el MQTT Broker de ALSTOM, consiguiendo implementar una serie de funcionalidades de la forma más óptima posible. Estas funcionalidades serán descritas en detalle a continuación.  

Sensortag como Actuador
-----------------------
El usuario tiene la posibilidad de necender o pagar remotamente tanto el led del Sensortag como el buzzer. Estas acciones no están ligadas a un Sensortag concreto sino que se ligan al material al que el Sensortag acompañará. Dado que hay una gran variedad de materiales y puede interesar cambiar el material al que va ligado un Sensortag, esta configuración de activación puede ser modificada remotamente.
### Actuación
Para activar/desactivar el led rojo del Sensortag se manda 1|0 al topic:  
`sala/material/leds`  
Para activar/desactivar el buzzer del Sensortag se manda 1|0 al topic:  
`sala/material/buzz`    
En ambos casos las longitudes de cada parámetro del topic está reglada. Esto es importante porque como se verá a contiuación el parámetro "material" es modificable. Las longitudes de los parámetros son las siguientes:  

+ sala: 3 caracteres 
+ material: 5 caracteres
+ leds/buzz: 4 caracteres  

La longitud total del topic es de 14 caracteres (e.g `A01/MAT00/leds`).

###Configuración 
Como ya se ha comentado el material al que responde el Sensortag puede ser modificado remotamente por el usuario. Para ello se debe mandar el nuevo material (IMP: longitud de 5 caracteres!!!!!!) al siguiente topic:  
`sala/client-id/Conf`  
Cuando el Sensortag recibe este mensaje modifica el parámetro del topic y lo guarda en memoria para evitar tener que ser reconfigurado si se apaga o reinicia de forma no progaramada.  
Es importante recordar que al igual que en el topic de actuación las longitudes de cada parámetro están fijadas siendo:  

+ sala: 3 caracteres
+ client-id: 14 caracteres (Identificación única del Sensortag)
+ Conf: 4 caracteres  

La longitud total del topic es de 23 caracteres. Un ejemplo de este topic podría ser: ` A01/d:00124b30bd87/Conf` con payload: `MAT01`. 


Publicación datos Sensores
--------------------------
###Publicación  
Los datos de los sensores son publicados cada cierto intervalo de tiempo según el siguiente topic:  
`sala/client-id/status/fmt/json`  
Para cada Sensortag se pueden seleccionar los sensores que se quieren leer y el intervalo de medida  para cada uno de ellos independientemente. El intervalo de publicación se obtendrá a partir del mínimo de los intervalos de medida para los sensores cuyo valor es publicado.  
Las longitudes de cada parámetro son fijas, siendo en cada caso:  

 + sala: 3 caracteres
 + client-id: 14 caracteres
 + status: 6 caracteres
 + fmt: 3 caracteres
 + json: 4 caracteres

La longitud del topic es de 34 caracteres (e.g `A01/d:00124b30bd87`). Como se indica en el topic los valores de los sensores se envían en formato JSON para su lectura de forma fácil y cómoda. 
### Selección de Sensores
Como se ha comentado, es posible seleccionar desde la nube las medidas de qué sensores serán publicados al broker. Para Activar la publicación se debe enviar "1" al siguiente topic:  
`sala/client-id/form-field/Pub`  
siendo el campo form-field un string representativo de cada tipo de sensor. Si se envía el mensaje "0", esta medida se dejará de publicar en el broker.Al contrario que en los anteriores topics descritos la longitud de este topic no es fija, pero siempre será superior a 28 caracteres (e.g `A01/d:00124b30bd87/light/Pub`).
### Modificación de intervalos de medida
Para modificar el intervalo de medida de un sensor en concreto es necesario enviar el numero de segundos del nuevo intervalo (sin decimales, solo acepta números enteros) al siguiente topic:  
`sala/client-id/form-field/Int`  
El formato seguido para modificar este parámetro es el mismo que para las publicaciones y dado que contiene el parámetro "form-field" la longitud de este topic tampoco es fija, pero siempre será mayor a 28 caracteres (e.g. `A01/d:00124b30bd87/light/Pub`).  
Hay que resaltar que cada vez que se modifica uno de estos parámetros sus valores osn guardados en la memoria Flash del dispositivo y son recargados si este es reiniciado.

Alarmas
-------
El Sensortag tiene también un servicio de alarmas asíncronas para alertar sobre eventos en los sensores o sobre la pulsación del boton por parte del usuario.
En el primer caso, la alarma se activa al sobrepasarse un límite impuesto para cada uno de los sensores. Este límite es configurable por el usuario como se describirá a continuación. En el segundo caso la alarma se activa cuando el usuario pulsa el botón del Sensortag para apagar el led rojo. Las alarmas son publicadas en un topic común, variando el mensaje según el tipo de alarma.  
`sala/client-id/alarm/SenOff`  
Este topic tiene una longitud fija total y por cada uno de sus parámetros:  

+ sala: 3 caracteres
+ client-id: 14 caracteres
+ alarm: 5 caracteres
+ SenOff: 6 caracteres  

La longitud total del topic es de 31 caracteres (e.g `A01/d:00124b30bd87/alarm/SenOff`).  
### Activación de las alarmas
Por defecto ninguna de las alarmas asociadas a los sensores están activas (solo está activa la alarma asociada al botón). Para activarlas o volverlas a desactivar se debe enviar "1|0" al topic:  
`sala/client-id/form_field/LimOn`  
Al formar parte de este topic el campo form_field no tiene un a longitud constante pero su longitud siempre supera los 27 caracteres como todos los topics que tienen este campo.

### Modificación de los límites
Los límites a partir de los cuáles se envía la alarma son configurables por el usuario. Para modificarlos solo hay que enviar el valor del límite al siguiente topic:  
`sala/client-id/form-field/Lim`  
Es importante resaltar que el mensaje aceptado por estos topics son valores enteros. Además este límite es comparado con el valor obtenido del sensor sin modificaciones. Ejemplo:  
    Si el valor de la medida tras ser procesada tiene dos decimales (e.g 45,39) y se desea que el              límite sea de 45,11 , este valor debe ser multiplicado por 100. El valor del límite sería: 4511.  
La longitud de este topic al contener el parámetro "form-filed" no esfija pero como ocurre con el topics Pub y Int nunca es inferior a 28 caracteres (e.g `A01/d:00124b30bd87/Light/Lim`).

Información
-----------
Dada la cantidad de inforamción referente a cada sensor que es almacenada en cada Sensortag se hace necesario implementar un método para enviar esta información desde el Sensortag a la nube. Para ello se usarán el topics de información asociados al Sensortag. Estos topics tienen la siguiente estructura:  
`sala/client-id/info/fmt/json` (1)  
`sala/client-id/Info`(2)  
El primero de los topics es el encargado de publicar los valores en el broker, mientras que el segundo da la orden de publicación al Sensortag. Es decir, para obtener la inforamción de un sensor se debe enviar "1" al topic (2). Al recibir el Sensortag este mensaje, enviará un mensaje al topic (1) con toda la información del dispositivo.  
Las longitudes de los topics son fijas siendo de (1) 28 caracteres y de (2) 23 caracteres.  
Un ejemplo del menaje recibido podría ser el siguiente:  
~~~
{"d":{"myName":"d:00124b30bd87","Seq #":2,"Uptime (sec)":39,  
"Def Route":"fe80::212:4b00:616:e25","RSSI (dBm)":134217728,  
"batmon_temp":{"Publish":0,"Interval":30,"Limit":4000},  
"batmon_volt":{"Publish":0,"Interval":30,"Limit":2250},  
"bmp_pres":{"Publish":0,"Interval":30,"Limit":20000},  
"bmp_temp":{"Publish":0,"Interval":30,"Limit":4000},  
"tmp_obj":{"Publish":`0,"Interval":30,"Limit":40000},  
"tmp_amb":{"Publish":0,"Interval":30,"Limit":40000},  
"light":{"Publish":0,"Interval":30,"Limit":5000},  
"hdc_hum":{"Publish":0,"Interval":30,"Limit":9000},  
"hdc_temp":{"Publish":0,"Interval":30,"Limit":4000}}}
~~~

MQTT Client & Mosquitto Broker
------------------------------
The device will try to connect to the broker over NAT64, so you will
need a NAT64 gateway in your network to make this work. If this is not an option for you, you can
configure the device to publish to a local MQTT broker over end-to-end IPv6.

For the SensorTag, changes to the MQTT configuration get saved in external
flash and persist across device restarts. The same does not hold true for
Srf+EM builds.

You can then use this to toggle LEDs. In order to public data using Mosquitto Broker, the terminal commands needed are the following:

`mosquitto_pub -h <broker IP> -m "1" -t sala/MAT00/leds`  
`mosquitto_pub -h <broker IP> -m "PIN02" -t sala/client_ID/Conf`

The message (-m) can change depending waht the user want to send.
Bear in mind that, even though the topic suggests that messages are of json
format, they are in fact not. This was done in order to avoid linking a json
parser into the firmware.
