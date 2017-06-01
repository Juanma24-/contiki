DEFINES+=PROJECT_CONF_H=\"project-conf.h\"
CONTIKI_PROJECT = cc26xx-demo

all: $(CONTIKI_PROJECT)

CFLAGS += -ffunction-sections
 
LDFLAGS += -Wl,--gc-sections,--undefined=_reset_vector__,--undefined=InterruptVectors,--undefined=_copy_data_init__,--undefined=_clear_bss_init__,--undefined=_end_of_init__

DISP_DRIVER_DIR = ./Display

DISP_DRIVER_FILES += Display.c DisplaySharp.c DisplayExt.c
DISP_DRIVER_FILES += SharpGrLib.c circle.c context.c image.c line.c rectangle.c string.c fontfixed6x8.c


PROJECTDIRS += $(DISP_DRIVER_DIR)
PROJECT_SOURCEFILES += $(DISP_DRIVER_FILES)

PROJECT_SOURCEFILES += splash_image.c
CONTIKI_WITH_IPV6 = 1

CONTIKI = ../../..
include $(CONTIKI)/Makefile.include
