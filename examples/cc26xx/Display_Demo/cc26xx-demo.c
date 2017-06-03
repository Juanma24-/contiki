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
/**
 * \addtogroup cc26xx-platforms
 * @{
 *
 * \defgroup cc26xx-examples CC26xx Example Projects
 *
 * Example projects for CC26xx-based platforms.
 * @{
 *
 * \defgroup cc26xx-demo CC26xx Demo Project
 *
 *   Example project demonstrating the CC13xx/CC26xx platforms
 *
 *   This example will work for the following boards:
 *   - srf06-cc26xx: SmartRF06EB + CC13xx/CC26xx EM
 *   - CC2650 and CC1350 SensorTag
 *   - CC1310, CC1350, CC2650 LaunchPads
 *
 *   This is an IPv6/RPL-enabled example. Thus, if you have a border router in
 *   your installation (same RDC layer, same PAN ID and RF channel), you should
 *   be able to ping6 this demo node.
 *
 *   This example also demonstrates CC26xx BLE operation. The process starts
 *   the BLE beacon daemon (implemented in the RF driver). The daemon will
 *   send out a BLE beacon periodically. Use any BLE-enabled application (e.g.
 *   LightBlue on OS X or the TI BLE Multitool smartphone app) and after a few
 *   seconds the cc26xx device will be discovered.
 *
 * - etimer/clock : Every CC26XX_DEMO_LOOP_INTERVAL clock ticks the LED defined
 *                  as CC26XX_DEMO_LEDS_PERIODIC will toggle and the device
 *                  will print out readings from some supported sensors
 * - sensors      : Some sensortag sensors are read asynchronously (see sensor
 *                  documentation). For those, this example will print out
 *                  readings in a staggered fashion at a random interval
 * - Buttons      : CC26XX_DEMO_SENSOR_1 button will toggle CC26XX_DEMO_LEDS_BUTTON
 *                - CC26XX_DEMO_SENSOR_2 turns on LEDS_REBOOT and causes a
 *                  watchdog reboot
 *                - The remaining buttons will just print something
 *                - The example also shows how to retrieve the duration of a
 *                  button press (in ticks). The driver will generate a
 *                  sensors_changed event upon button release
 * - Reed Relay   : Will toggle the sensortag buzzer on/off
 *
 * @{
 *
 * \file
 *     Example demonstrating the cc26xx platforms
 */
#include "contiki.h"


#include "board-peripherals.h"
#include "dev/leds.h"
#include <stdio.h>

#include "ti-lib.h"
#include <stdint.h>
#include <Display/Display.h>
#include <Display/DisplayExt.h>

/* Example GrLib image */
#include "splash_image.h"
/*---------------------------------------------------------------------------*/
PROCESS(display_demo_process, "Display demo process");
AUTOSTART_PROCESSES(&display_demo_process);
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(display_demo_process, ev, data)
{

  PROCESS_BEGIN();

    printf("Display demo\n");
    unsigned int ledPinValue = 0;

    /* Initialize display and try to open both UART and LCD types of display. */
    Display_Params params;
    Display_Params_init(&params);
    params.lineClearMode = DISPLAY_CLEAR_BOTH;

    /* Open both an available LCD display and an UART display.
     * Whether the open call is successful depends on what is present in the
     * Display_config[] array of the board file.
     *
     * Note that for SensorTag evaluation boards combined with the SHARP96x96
     * Watch DevPack, there is a pin conflict with UART such that one must be
     * excluded, and UART is preferred by default. To display on the Watch
     * DevPack, add the precompiler define BOARD_DISPLAY_EXCLUDE_UART.
     */
    Display_Handle hDisplayLcd = Display_open(Display_Type_LCD, &params);
    
    IOCPortConfigureSet(IOID_15, IOC_PORT_GPIO, IOC_STD_OUTPUT);
    GPIO_setOutputEnableDio(IOID_15,GPIO_OUTPUT_ENABLE);

    /* Check if the selected Display type was found and successfully opened */
    if (hDisplayLcd) {
      Display_print0(hDisplayLcd, 5, 3, "Hello LCD!");

      /* Wait a while so text can be viewed. */
      clock_wait(20*CLOCK_SECOND);

      /*
       * Use the GrLib extension to get the GraphicsLib context object of the
       * LCD, if it is supported by the display type.
      */
      tContext *pContext = DisplayExt_getGrlibContext(hDisplayLcd);

      /* It's possible that no compatible display is available. */
      if (pContext) {
        /* Draw splash */
        GrImageDraw(pContext, &splashImage, 0, 0);
        GrFlush(pContext);
      }
      else {
        /* Not all displays have a GraphicsLib back-end */
        Display_print0(hDisplayLcd, 0, 0, "Display driver");
        Display_print0(hDisplayLcd, 1, 0, "is not");
        Display_print0(hDisplayLcd, 2, 0, "GrLib capable!");
      }

      /* Wait for a bit, then clear */
      clock_wait(2 * CLOCK_SECOND);
      Display_clear(hDisplayLcd);
    }

    /* Loop forever, alternating LED state and Display output. */
    while (1) { 
      if (hDisplayLcd) {
        /* Print to LCD and clear alternate lines if the LED is on or not. */
        Display_clearLine(hDisplayLcd, ledPinValue ? 0:1);
        Display_print1(hDisplayLcd, ledPinValue ? 1:0, 0, "LED: %s",(!ledPinValue) ? "On!":"Off!");
      }else{
        leds_on(LEDS_RED);
      }
      if(ledPinValue){
        //leds_on(LEDS_GREEN);
        GPIO_writeDio(IOID_15, 1);
        printf("LED ON!!\n");
        ledPinValue = 1;
      }else{
        //leds_off(LEDS_GREEN);
        GPIO_writeDio(IOID_15, 0);
        printf("LED OFF!!\n");
        ledPinValue = 0;
      }
      clock_wait(5*CLOCK_SECOND);
    }  
  
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
/**
 * @}
 * @}
 * @}
 */
