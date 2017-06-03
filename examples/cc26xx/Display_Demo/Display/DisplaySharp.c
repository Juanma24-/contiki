/*
 * Copyright (c) 2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* -----------------------------------------------------------------------------
 *  Includes
 * ------------------------------------------------------------------------------
 */
// TI RTOS drivers
#include <driverlib/ioc.h>
#include <driverlib/gpio.h>
#include <common/board-spi.h>

#include <Display/grlib.h>
#include <Display/Display.h>
#include <Display/DisplaySharp.h>
#include <Display/SharpGrLib.h>
#include <board.h>
#include <string.h>
#include <stdio.h>


/* -----------------------------------------------------------------------------
 *  Constants and macros
 * ------------------------------------------------------------------------------
 */
// Timeout of semaphore that controls exclusive to the LCD (infinite)
#define ACCESS_TIMEOUT    BIOS_WAIT_FOREVER

/// Pin ID used to indicate no pin
#define PIN_UNASSIGNED              0xFF
/// Pin ID used to terminate a list of PIN_Id or PIN_Config entries
#define PIN_TERMINATE               0xFE

#define SPI_STATUS_UNDEFINEDCMD   -2

/* -----------------------------------------------------------------------------
 *   Type definitions
 * ------------------------------------------------------------------------------
 */
#define IOC_LCD_OUTPUT_0          (IOC_CURRENT_4MA | IOC_STRENGTH_MAX |      \
                                 IOC_IOPULL_DOWN | IOC_SLEW_DISABLE |         \
                                 IOC_HYST_DISABLE | IOC_NO_EDGE |           \
                                 IOC_INT_DISABLE | IOC_IOMODE_NORMAL |      \
                                 IOC_NO_WAKE_UP | IOC_INPUT_DISABLE )

#define IOC_LCD_OUTPUT_1          (IOC_CURRENT_4MA | IOC_STRENGTH_MAX |      \
                                 IOC_IOPULL_UP | IOC_SLEW_DISABLE |         \
                                 IOC_HYST_DISABLE | IOC_NO_EDGE |           \
                                 IOC_INT_DISABLE | IOC_IOMODE_NORMAL |      \
                                 IOC_NO_WAKE_UP | IOC_INPUT_DISABLE )


/* -----------------------------------------------------------------------------
 *                           Local variables
 * ------------------------------------------------------------------------------
 */
/* Display function table for sharp implementation */
const Display_FxnTable DisplaySharp_fxnTable = {
    DisplaySharp_open,
    DisplaySharp_clear,
    DisplaySharp_clearLines,
    DisplaySharp_put5,
    DisplaySharp_close,
    DisplaySharp_control,
    DisplaySharp_getType,
};
//static struct pt_sem mutex;
/* -----------------------------------------------------------------------------
 *                                          Functions
 * ------------------------------------------------------------------------------
 */
/*!
 * @fn          DisplaySharp_open
 *
 * @brief       Initialize the LCD
 *
 * @descr       Initializes the pins used by the LCD, creates resource access
 *              protection semaphore, turns on the LCD device, initializes the
 *              frame buffer, initializes to white background/dark foreground,
 *              and finally clears the object->displayColor.
 *
 * @param       hDisplay - pointer to Display_Config struct
 * @param       params - display parameters
 *
 * @return      Pointer to Display_Config struct
 */
Display_Handle DisplaySharp_open(Display_Handle hDisplay,Display_Params *params)
{
    DisplaySharp_HWAttrs *hwAttrs = (DisplaySharp_HWAttrs *)hDisplay->hwAttrs;
    DisplaySharp_Object  *object  = (DisplaySharp_Object  *)hDisplay->object;

    object->lineClearMode = params->lineClearMode;

    if (hwAttrs->csPin != PIN_TERMINATE)
    {
        IOCPortConfigureSet(hwAttrs->csPin, IOC_PORT_GPIO, IOC_LCD_OUTPUT_0);
        GPIO_setOutputEnableDio(hwAttrs->csPin,GPIO_OUTPUT_ENABLE);
    }
    if (hwAttrs->extcominPin != PIN_TERMINATE)
    {
        IOCPortConfigureSet(hwAttrs->extcominPin, IOC_PORT_GPIO, IOC_LCD_OUTPUT_0);
        GPIO_setOutputEnableDio(hwAttrs->extcominPin,GPIO_OUTPUT_ENABLE);
    }
    if (hwAttrs->powerPin != PIN_TERMINATE)
    {
        IOCPortConfigureSet(hwAttrs->powerPin , IOC_PORT_GPIO, IOC_LCD_OUTPUT_1);
        GPIO_setOutputEnableDio(hwAttrs->powerPin,GPIO_OUTPUT_ENABLE);
    }
    if (hwAttrs->enablePin != PIN_TERMINATE)
    {
        IOCPortConfigureSet(hwAttrs->enablePin, IOC_PORT_GPIO, IOC_LCD_OUTPUT_1);
        GPIO_setOutputEnableDio(hwAttrs->enablePin,GPIO_OUTPUT_ENABLE);
        GPIO_writeDio(hwAttrs->enablePin,1);
    }
    
    board_spi_open(4000000, BOARD_IOID_SPI_SCK);

    // Init colors
    object->displayColor.bg = ClrBlack;
    object->displayColor.fg = ClrWhite;

    // Initialize the GrLib back-end transport
    SharpGrLib_init(hwAttrs->csPin);

    object->g_sDisplay.lSize         = sizeof(tDisplay);
    object->g_sDisplay.pFxns         = &g_sharpFxns;
    object->g_sDisplay.pvDisplayData = object->displayBuffer;
    object->g_sDisplay.usHeight      = hwAttrs->pixelHeight;
    object->g_sDisplay.usWidth       = hwAttrs->pixelWidth;
    object->g_sDisplay.pvDisplayData = hwAttrs->displayBuf;

    // Graphics library init
    GrContextInit(&object->g_sContext, &object->g_sDisplay, &g_sharpFxns);

    // Graphics properties
    GrContextForegroundSet(&object->g_sContext, object->displayColor.fg);
    GrContextBackgroundSet(&object->g_sContext, object->displayColor.bg);
    GrContextFontSet(&object->g_sContext, &g_sFontFixed6x8);

    // Clear display
    GrClearDisplay(&object->g_sContext);
    GrFlush(&object->g_sContext);

    return hDisplay;
}


/*!
 * @fn          DisplaySharp_clear
 *
 * @brief       Clears the display
 *
 * @param       hDisplay - pointer to Display_Config struct
 *
 * @return      void
 */
void DisplaySharp_clear(Display_Handle hDisplay)
{
    DisplaySharp_Object *object = (DisplaySharp_Object  *)hDisplay->object;
    
    GrClearDisplay(&object->g_sContext);
    GrFlush(&object->g_sContext);

}


/*!
 * @fn          DisplaySharp_clearLines
 *
 * @brief       Clears lines lineFrom-lineTo of the display, inclusive
 *
 * @param       hDisplay - pointer to Display_Config struct
 * @param       lineFrom - line index (0 .. )
 * @param       lineTo - line index (0 .. )
 *
 * @return      void
 */
void DisplaySharp_clearLines(Display_Handle hDisplay,uint8_t lineFrom, uint8_t lineTo)
{
    DisplaySharp_Object *object = (DisplaySharp_Object  *)hDisplay->object;

    if (lineTo <= lineFrom)
    {
        lineTo = lineFrom + 1;
    }

    tRectangle rect = {
        .sXMin =                                                     0,
        .sXMax = object->g_sContext.sClipRegion.sXMax,
        .sYMin = lineFrom * object->g_sContext.pFont->ucHeight,
        .sYMax = (lineTo + 1) * object->g_sContext.pFont->ucHeight - 1,
    };

    GrContextForegroundSet(&object->g_sContext, object->displayColor.bg);
    GrRectFill(&object->g_sContext, &rect);
    GrContextForegroundSet(&object->g_sContext, object->displayColor.fg);
    GrFlush(&object->g_sContext);
}


/*!
 * @fn          DisplaySharp_put5
 *
 * @brief       Write a text string to a specific line/column of the display
 *
 * @param       hDisplay - pointer to Display_Config struct
 * @param       line - line index (0..)
 * @param       column - column index (0..)
 * @param       fmt - format string
 * @param       aN - optional format arguments
 *
 * @return      void
 */
void DisplaySharp_put5(Display_Handle hDisplay, uint8_t line,
                       uint8_t column, uintptr_t fmt, uintptr_t a0,
                       uintptr_t a1, uintptr_t a2, uintptr_t a3, uintptr_t a4)
{
    DisplaySharp_Object *object = (DisplaySharp_Object  *)hDisplay->object;

    uint8_t xp, yp, clearStartX, clearEndX;

    char    dispStr[23];

        xp          = column * object->g_sContext.pFont->ucMaxWidth + 1;
        yp          = line * object->g_sContext.pFont->ucHeight + 0;
        clearStartX = clearEndX = xp;

        switch (object->lineClearMode)
        {
            case DISPLAY_CLEAR_LEFT:
                clearStartX = 0;
                break;
            case DISPLAY_CLEAR_RIGHT:
                clearEndX = object->g_sContext.sClipRegion.sXMax;
                break;
            case DISPLAY_CLEAR_BOTH:
                clearStartX = 0;
                clearEndX   = object->g_sContext.sClipRegion.sXMax;
                break;
            case DISPLAY_CLEAR_NONE:
                default:
                break;
        }

        if (clearStartX != clearEndX)
        {
            tRectangle rect = {
                .sXMin = clearStartX,
                .sXMax = clearEndX,
                .sYMin = yp,
                .sYMax = yp + object->g_sContext.pFont->ucHeight - 1,
            };

            GrContextForegroundSet(&object->g_sContext, object->displayColor.bg);
            GrRectFill(&object->g_sContext, &rect);
            GrContextForegroundSet(&object->g_sContext, object->displayColor.fg);
        }

        snprintf(dispStr, sizeof(dispStr), (const char*)fmt, a0, a1, a2, a3, a4);

        // Draw a text on the display
        GrStringDraw(&object->g_sContext,dispStr,AUTO_STRING_LENGTH,xp,yp,OPAQUE_TEXT);
        GrFlush(&object->g_sContext);

}


/*!
 * @fn          DisplaySharp_close
 *
 * @brief       Turns of the display and releases the LCD control pins
 *
 * @param       hDisplay - pointer to Display_Config struct
 *
 * @return      void
 */
void DisplaySharp_close(Display_Handle hDisplay)
{
    DisplaySharp_HWAttrs *hwAttrs = (DisplaySharp_HWAttrs *)hDisplay->hwAttrs;

        // Turn off the display
        if (hwAttrs->enablePin != PIN_TERMINATE){
            GPIO_writeDio(hwAttrs->enablePin, 0);
        }
        // Release resources
        if (hwAttrs->csPin != PIN_TERMINATE){
            GPIO_setOutputEnableDio(hwAttrs->csPin,GPIO_OUTPUT_DISABLE);
        }
        if (hwAttrs->extcominPin != PIN_TERMINATE){
            GPIO_setOutputEnableDio(hwAttrs->extcominPin,GPIO_OUTPUT_DISABLE);   
        }
        if (hwAttrs->powerPin != PIN_TERMINATE){
            GPIO_setOutputEnableDio(hwAttrs->powerPin,GPIO_OUTPUT_DISABLE);    
        }
        if (hwAttrs->enablePin != PIN_TERMINATE){
            GPIO_setOutputEnableDio(hwAttrs->enablePin,GPIO_OUTPUT_DISABLE);
        }
        board_spi_close();

        // Deconfigure GrLib back-end
        SharpGrLib_init(PIN_UNASSIGNED);

        // Release LCD
}

/*!
 * @fn          DisplaySharp_control
 *
 * @brief       Function for setting control parameters of the Display driver
 *              after it has been opened.
 *
 * @param       hDisplay - pointer to Display_Config struct
 * @param       cmd - command to execute, supported commands are:
 *              | Command                        | Description             |
 *              |------------------------------- |-------------------------|
 *              | ::DISPLAY_CMD_TRANSPORT_CLOSE  | Close SPI but leave control pins |
 *              | ::DISPLAY_CMD_TRANSPORT_OPEN   | Re-open SPI driver      |
 * @param       arg - argument to the command
 *
 * @return      ::DISPLAY_STATUS_SUCCESS if success, or error code if error.
 */
int DisplaySharp_control(Display_Handle hDisplay, unsigned int cmd, void *arg)
{
    DisplaySharp_HWAttrs *hwAttrs = (DisplaySharp_HWAttrs *)hDisplay->hwAttrs;
    DisplaySharp_Object  *object  = (DisplaySharp_Object  *)hDisplay->object;

    /* Initialize return value */
    int ret = DISPLAY_STATUS_ERROR;

    /* Perform command */
    switch(cmd)
    {
        case DISPLAY_CMD_TRANSPORT_CLOSE:
                    // Close SPI and tell back-end there is no SPI
                    board_spi_close();
                    SharpGrLib_init(hwAttrs->csPin);
                    ret = DISPLAY_STATUS_SUCCESS;
            break;

        case DISPLAY_CMD_TRANSPORT_OPEN:
                    // Re-open SPI and re-init back-end
                    board_spi_open(4000000, BOARD_IOID_SPI_SCK);
                    SharpGrLib_init(hwAttrs->csPin);
                    ret = DISPLAY_STATUS_SUCCESS;
            break;

        case DISPLAYSHARP_CMD_SET_COLORS:

                object->displayColor = *(DisplaySharpColor_t *)arg;

                GrContextForegroundSet(&object->g_sContext, object->displayColor.fg);
                GrContextBackgroundSet(&object->g_sContext, object->displayColor.bg);
                // Return success
                ret = DISPLAY_STATUS_SUCCESS;
            break;

        default:
            /* The command is not defined */
            ret = SPI_STATUS_UNDEFINEDCMD;
            break;
    }

    return ret;
}

/*!
 * @fn          DisplaySharp_getType
 *
 * @brief       Returns type of transport
 *
 * @return      Display type define LCD
 */
unsigned int DisplaySharp_getType(void)
{
    return Display_Type_LCD | Display_Type_GRLIB;
}
