DEFINES+=PROJECT_CONF_H=\"project-conf.h\"

all: alstom-mqtt-iot


PROJECT_SOURCEFILES += mqtt-client.c

CONTIKI_WITH_IPV6 = 1
APPS += mqtt

DISP_DRIVER_DIR = ./Display
DISP_DRIVER_FILES += Display.c DisplaySharp.c DisplayExt.c
DISP_DRIVER_FILES += SharpGrLib.c circle.c context.c image.c line.c rectangle.c string.c fontfixed6x8.c
PROJECTDIRS += $(DISP_DRIVER_DIR)
PROJECT_SOURCEFILES += $(DISP_DRIVER_FILES)
PROJECT_SOURCEFILES += splash_image.c


CONTIKI=../..
include $(CONTIKI)/Makefile.include
