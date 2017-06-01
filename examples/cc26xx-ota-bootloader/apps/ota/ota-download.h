/** @file   ota-download.h
 *  @brief  OTA Image Download Mechanism
 *  @author Mark Solters <msolters@gmail.com>
 */

#ifndef OTA_DOWNLOAD_H
#define OTA_DOWNLOAD_H

#include "contiki.h"
#include "contiki-lib.h"
#include "contiki-net.h"
#include "ip64-addr.h"
#include "er-coap-engine.h"
#include "ti-lib.h"
#include "ota.h"

static uip_ipaddr_t ota_server_ipaddr;
#define OTA_SERVER_IP() uip_ip6addr(&ota_server_ipaddr, 0, 0, 0, 0, 0, 0xffff, 0xc0a8, 0x012f)

/* OTA Download Thread */
extern struct process *ota_download_th_p; // Pointer to OTA Download thread


#define OTA_BUFFER_SIZE 1024

#endif
