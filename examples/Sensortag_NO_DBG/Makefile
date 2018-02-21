DEFINES+=PROJECT_CONF_H=\"project-conf.h\"

all: alstom-mqtt-iot


PROJECT_SOURCEFILES += mqtt-client.c

CONTIKI_WITH_IPV6 = 1
APPS += mqtt

CFLAGS += -ffunction-sections
LDFLAGS += -Wl,--gc-sections,--undefined=_reset_vector__,--undefined=InterruptVectors,--undefined=_copy_data_init__,--undefined=_clear_bss_init__,--undefined=_end_of_init__

CONTIKI=../..
include $(CONTIKI)/Makefile.include
