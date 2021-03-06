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
 * -----------------------------------------------------------------------------
 */

#include <Display/Display.h>
#include <Display/DisplaySharp.h>
#include <stdio.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <board.h>
#include <leds.h>

/* -----------------------------------------------------------------------------
 *  Externs
 * -----------------------------------------------------------------------------
 */
#define BOARD_DISPLAY_SHARP_SIZE    96 // 96->96x96 is the most common board, alternative is 128->128x128.

DisplaySharp_Object displaySharpObject;

static uint8_t sharpDisplayBuf[BOARD_DISPLAY_SHARP_SIZE * BOARD_DISPLAY_SHARP_SIZE / 8];

const DisplaySharp_HWAttrs displaySharpHWattrs = {
    .csPin       = BOARD_IOID_DEVPACK_CS,
    .extcominPin = BOARD_IOID_DEVPK_LCD_EXTCOMIN,
    .powerPin    = 0xFE,
    .enablePin   = BOARD_IOID_DEVPK_LCD_ENABLE,
    .pixelWidth  = BOARD_DISPLAY_SHARP_SIZE,
    .pixelHeight = BOARD_DISPLAY_SHARP_SIZE,
    .displayBuf  = sharpDisplayBuf,
};
Display_Config Display_config[] = {
    {
        .fxnTablePtr = &DisplaySharp_fxnTable,
        .object      = &displaySharpObject,
        .hwAttrs     = &displaySharpHWattrs
    },
    { NULL, NULL, NULL } // Terminator
};

/* -----------------------------------------------------------------------------
 *  Constants and macros
 * -----------------------------------------------------------------------------
 */

/* -----------------------------------------------------------------------------
 *   Type definitions
 * -----------------------------------------------------------------------------
 */

/* -----------------------------------------------------------------------------
 *                           Local variables
 * -----------------------------------------------------------------------------
 */
/* Default Display parameters structure */
const Display_Params Display_defaultParams = {
    DISPLAY_CLEAR_BOTH,   /* Clear entire line before writing */
};


/* -----------------------------------------------------------------------------
 *                                          Functions
 * -----------------------------------------------------------------------------
 */

/*
 *  ======== Display_doOpen ========
 */
Display_Handle Display_doOpen(uint32_t id, Display_Params *params)
{
    Display_Handle handle;
    if (params == NULL)
    {
        params = (Display_Params *)&Display_defaultParams;
    }

    /* Call each driver's open function */
    uint32_t i;
    for (i = 0; Display_config[i].fxnTablePtr != NULL; i++)
    {
        handle = (Display_Handle)&Display_config[i];

        /* Open if id matches, or if meta-type matches */
        if ((handle->fxnTablePtr->getTypeFxn() & id))
        {
            if (NULL == handle->fxnTablePtr->openFxn(handle, params))
            {
                // Couldn't open. Continue trying to open in case a type was
                // provided and there's more than one of the type.
                continue;
            }
            else
            {
                // Return first matching
                return handle;
            }
        }
    }
    // Couldn't open.
    leds_on(LEDS_RED);
    printf("Couldn't open selected Displays\n");
    return NULL;
}

/*
 *  ======== Display_doParamsInit ========
 */
void Display_doParamsInit(Display_Params *params)
{
    *params = Display_defaultParams;
}


/*
 *  ======== Display_doClear ========
 */
void Display_doClear(Display_Handle handle)
{
    if (NULL == handle)
    {
        leds_on(LEDS_RED);
        printf("Trying to use NULL-handle.\n");
        return;
    }

    handle->fxnTablePtr->clearFxn(handle);
}


/*
 *  ======== Display_doClearLines ========
 */
void Display_doClearLines(Display_Handle handle, uint8_t fromLine, uint8_t toLine)
{
    if (NULL == handle)
    {
        leds_on(LEDS_RED);
        printf("Trying to use NULL-handle.\n");
        return;
    }

    handle->fxnTablePtr->clearLinesFxn(handle, fromLine, toLine);
}


/*
 *  ======== Display_doPut5 ========
 */
void Display_doPut5(Display_Handle handle, uint8_t line, uint8_t column,
                    uintptr_t fmt, uintptr_t a0, uintptr_t a1, uintptr_t a2,
                    uintptr_t a3, uintptr_t a4)
{
    if (NULL == handle)
    {
        printf("Trying to use NULL-handle.\n");
        return;
    }

    handle->fxnTablePtr->put5Fxn(handle, line, column, fmt, a0, a1, a2,
                                 a3, a4);
}

/*
 *  ======== Display_doControl ========
 */
void  Display_doControl(Display_Handle handle, unsigned int cmd, void *arg)
{
    if (NULL == handle)
    {
        leds_on(LEDS_RED);
        printf("Trying to use NULL-handle.\n");
        return;
    }

    handle->fxnTablePtr->controlFxn(handle, cmd, arg);
}

/*
 *  ======== Display_doClose ========
 */
void Display_doClose(Display_Handle handle)
{
    if (NULL == handle)
    {
        leds_on(LEDS_RED);
        printf("Trying to use NULL-handle.\n");
        return;
    }

    handle->fxnTablePtr->closeFxn(handle);
}
/*
 *  ======= Display_move =======
 */
uint8_t Display_move(Display_Handle handle, char* string, uint8_t line)
{
    if(NULL == string){
        return line;
    }
    
    uint8_t offset = strlen(string)/16 + 1;
    line += offset;

    if(line>11){
        Display_clear(handle);
        return 0;
    }else{
        return line;
    }
    
    
}
