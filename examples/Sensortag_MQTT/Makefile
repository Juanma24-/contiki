DEFINES+=PROJECT_CONF_H=\"project-conf.h\"

all: cc26xx-web-demo




PROJECTDIRS += $(REST_RESOURCES_DIR)
PROJECT_SOURCEFILES += $(REST_RESOURCES_FILES)

PROJECT_SOURCEFILES += mqtt-client.c
PROJECT_SOURCEFILES += httpd-simple.c

CONTIKI_WITH_IPV6 = 1

# REST Engine shall use Erbium CoAP implementation
APPS += mqtt

CONTIKI=../..
include $(CONTIKI)/Makefile.include
