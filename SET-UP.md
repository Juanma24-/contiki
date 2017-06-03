The Contiki Operating System
============================

[![Build Status](https://travis-ci.org/contiki-os/contiki.svg?branch=master)](https://travis-ci.org/contiki-os/contiki/branches)

Contiki es un sistema operativo open source que corre en pequeños microcontroladores de bajo consumo y hace posible desarrollar aplicaciones que hacen un uso eficiente del hardware while provee conexiones wireless de bajo consumo a un gran rango de plataformas.

Para más información, vea the Contiki web<:

[http://contiki-os.org](http://contiki-os.org)

SET-UP
======
__Todo el proceso se ha realizado en Ubuntu 17.04__  
  
Para descargar la copia de Contiki de Contiki que contiene el firmware de demostración bajo Sensortag se recomienda hacer una copia del repositorio alojado en GitHub mediante los siguientes comandos ejecutados en terminal:  
~~~
git clone https://github.com/Juanma24-/contiki.git
cd contiki
git submodule update --init --recursive
~~~
(si no tiene instalado git, puede hacerlo mediante el siguiente comando: `git apt-get install git`).
A continuación es necesario instalar el toolchain con las herramientas para compilar los programas creados en Contiki:  
~~~
sudo apt-get install build-essential binutils-msp430 gcc-msp430 msp430-libc binutils-avr gcc-avr gdb-avr avr-libc avrdude binutils-arm-none-eabi gcc-arm-none-eabi gdb-arm-none-eabi openjdk-8-jdk openjdk-7-jre ant libncurses5-dev doxygen srecord  
~~~

MAKE SENSORTAG
==============

Para compilar el firmware realizado para Sensortag hay que situarse en la carpeta que contiene el archivo _Makefile_ del firmware, abrir un nuevo terminal y llamar a siguiente instrucción:  
~~~
make *firmware* TARGET=srf06-cc26xx BOARD=sensortag/cc2650 CPU_FAMILY=cc26xx
~~~
Sustituyendo la palabra _firmware_ por el nombre del archivo base del firmware.

FLASH SENSORTAG
===============

Para grabar el archivo compilado en el Sensortag se hace uso de la herramienta [http://processors.wiki.ti.com/index.php/Category:CCS_UniFlash](UniFlash). Una vez abierta se selecciona el dispositivo _CC2650F128_ y la conexión _XDX110 USB Debug Probe_ .  
Los pasos para realizar cargar el firmware son:  
1- Borrar la memoria Flash: Settings & Utilities -> Erase Entire Flash  
2- Cargar la nueva imagen: Program -> Browse -> Selecciona archivo .hex a cargar -> Load Image
