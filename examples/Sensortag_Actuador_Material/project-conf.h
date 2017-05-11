/*
 * Copyright (c) 2014, Texas Instruments Incorporated - http://www.ti.com/
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*---------------------------------------------------------------------------*/
#ifndef PROJECT_CONF_H_
#define PROJECT_CONF_H_
/*---------------------------------------------------------------------------*/
/* Change to match your configuration */
#define IEEE802154_CONF_PANID            0xABCD
#define RF_CORE_CONF_CHANNEL                 25
#define RF_BLE_CONF_ENABLED                   0
/*---------------------------------------------------------------------------*/
/* Enable the ROM bootloader */
#define ROM_BOOTLOADER_ENABLE                 1
#define BUTTON_SENSOR_CONF_ENABLE_SHUTDOWN    1
/*---------------------------------------------------------------------------*/
/*
 * Shrink the size of the uIP buffer, routing table and ND cache.
 * Set the TCP MSS
 */
#define UIP_CONF_BUFFER_SIZE                900
#define NBR_TABLE_CONF_MAX_NEIGHBORS          5
#define UIP_CONF_MAX_ROUTES                   5
#define UIP_CONF_TCP_MSS                    128
/*---------------------------------------------------------------------------*/
/*Activación de noncoredriver como capa de seguridad	
#undef LLSEC802154_CONF_ENABLED
#define LLSEC802154_CONF_ENABLED          1
#undef NETSTACK_CONF_FRAMER
#define NETSTACK_CONF_FRAMER              noncoresec_framer
#undef NETSTACK_CONF_LLSEC
#define NETSTACK_CONF_LLSEC               noncoresec_driver
#undef NONCORESEC_CONF_SEC_LVL
#define NONCORESEC_CONF_SEC_LVL           1
#define NONCORESEC_CONF_KEY { 0x00 , 0x01 , 0x02 , 0x03 , \
                              0x04 , 0x05 , 0x06 , 0x07 , \
                              0x08 , 0x09 , 0x0A , 0x0B , \
                              0x0C , 0x0D , 0x0E , 0x0F } 
*/                              

/*---------------------------------------------------------------------------*/
#endif /* PROJECT_CONF_H_ */
/*---------------------------------------------------------------------------*/
