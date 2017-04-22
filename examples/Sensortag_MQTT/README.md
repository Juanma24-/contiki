CC26xx Web Demo Readme
======================
OBJETIVOS
----------
El objetivo es obtener conectividad con el MQTT Broker de ALSTOM, eliminando  
toda funcionalidad de HTTP Server. Además se persigue añadir un modo de bajo  
consumo, gracias al uso de QoS en nivel 1 o 2. 

This demo project combines a number of web-based applications aiming to
demonstrate the CC26xx capability. The applications are:

* An MQTT client

IBM Quickstart / MQTT Client
----------------------------
The MQTT client can be used to:

* Publish sensor readings to an MQTT broker.
* Subscribe to a topic and as a result receive commands from an MQTT broker

The device will try to connect to IBM's quickstart over NAT64, so you will
need a NAT64 gateway in your network to make this work. A guide on how to
setup NAT64 is out of scope here. If this is not an option for you, you can
configure the device to publish to a local MQTT broker over end-to-end IPv6.
See below on how to change the destination broker's address.

By default the device will publish readings to IBM's quickstart service. The
publish messages include sensor readings but also some other information such
as device uptime in seconds and a message sequence number. Click the "IBM
Quickstart" link in the web page to go directly to your device's page
on Quickstart. After a few seconds, you will see something like this:

![A SensorTag on IBM Quickstart](img/quickstart-sensortag.png)

Sensor readings are only published if they have changed since the previous
reading (BatMon is an exception and always gets published). Additionally, you
can turn on/off individual readings from the config.html web page, as per the
figure below.

![Sensor Readings Configuration](img/sensor-readings-config.png)

Some of the MQTT client functionality can be configured even further:

* You can change the broker IP and port. This is useful if you want to use your
  own MQTT broker instead of IBM's quickstart. The example has been tested
  successfully with [mosquitto](http://mosquitto.org/)
* You can change the publish interval. Recommended values are 10secs or higher.
  You will not be allowed to set this to anything less than 5 seconds.
* If you want to use IBM's cloud service with a registered device, change
  'Org ID' and provide an 'Auth Token', which acts as a 'password', but bear in
  mind that it gets transported in clear text, both over the web configuration
  page as well as inside MQTT messages.
* The remaining configuration options are related to the content of MQTT
  messages and in general you won't have to modify them.

For the SensorTag, changes to the MQTT configuration get saved in external
flash and persist across device restarts. The same does not hold true for
Srf+EM builds.

You can also subscribe to topics and receive commands, but this will only
work if you use "Org ID" != 'quickstart'. Thus, if you provide a different
Org ID (do not forget the auth token!), the device will subscribe to:

`iot-2/cmd/+/fmt/json`

You can then use this to toggle LEDs or to turn the buzzer on and off.
The buzzer is only available on the SensorTag. To do this, you can for example
use mosquitto client to publish to `iot-2/cmd/leds/fmt/json`. So, to turn
the buzzer on, you would do this:

`mosquitto_pub -h <broker IP> -m "1" -t sala/tipo_op/leds`
`mosquitto_pub -h <broker IP> -m "1" -t sala/client_ID/Op_Mask`

Where `broker IP` should be replaced with the IP address of your mosquitto
broker (the one where you device has subscribed). Replace `-m "1'` with `-m "0"`
to turn the buzzer back off. Replace `buzz` with `leds` in the topic to change
the state of the LED.

Bear in mind that, even though the topic suggests that messages are of json
format, they are in fact not. This was done in order to avoid linking a json
parser into the firmware.
