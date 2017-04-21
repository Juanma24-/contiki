DEFINES+=PROJECT_CONF_H=\"project-conf.h\"

all: cc26xx-web-demo


PROJECT_SOURCEFILES += mqtt-client.c
PROJECT_SOURCEFILES += httpd-simple.c

CONTIKI_WITH_IPV6 = 1
APPS += mqtt

CONTIKI=../..
include $(CONTIKI)/Makefile.include
