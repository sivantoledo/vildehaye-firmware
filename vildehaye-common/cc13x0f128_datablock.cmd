/*
 * Copyright (c) 2015-2016, Texas Instruments Incorporated
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
/*
 *  ======== CC1310_LAUNCHXL.cmd ========
 *  CC26x0F128 PG2 linker configuration file for Code Composer Studio
 */

/* Override default entry point.                                             */
--entry_point ResetISR
/* Allow main() to take args                                                 */
--args 0x8
/* Suppress warnings and errors:                                             */
/* - 10063: Warning about entry point not being _c_int00                     */
/* - 16011, 16012: 8-byte alignment errors. Observed when linking in object  */
/*   files compiled using Keil (ARM compiler)                                */
--diag_suppress=10063,16011,16012

/* The starting address of the application.  Normally the interrupt vectors  */
/* must be located at the beginning of the application.                      */
#define FLASH_BASE              0x0
#define FLASH_SIZE              0x20000
#define RAM_BASE                0x20000000
#define RAM_SIZE                0x5000


#define FLASH_PAGE_SIZE         0x1000
#define FLASH_ADDR              0x0
#define FLASH_SIZE              0x20000
#define FLASH_APPL_ADDR         FLASH_ADDR
#define FLASH_APPL_SIZE         (24 * FLASH_PAGE_SIZE)              // 30 pages for the application

#define FLASH_LOG_ADDR          (FLASH_APPL_ADDR + FLASH_APPL_SIZE)
#define FLASH_LOG_SIZE          (6 * FLASH_PAGE_SIZE)              // 30 pages for the application

#define FLASH_DATA_ADDR         (FLASH_LOG_ADDR + FLASH_LOG_SIZE)
#define FLASH_DATA_SIZE         (1 * FLASH_PAGE_SIZE)               // 1 page for the config data

#define FLASH_CCFG_ADDR         (FLASH_DATA_ADDR + FLASH_DATA_SIZE)
#define FLASH_CCFG_SIZE         (1 * FLASH_PAGE_SIZE)               // 1 page for the ccfg

#define RAM_ADDR                0x20000000
#define RAM_SIZE                0x5000

__FLASH_ADDR      = FLASH_ADDR;
__FLASH_SIZE      = FLASH_SIZE;
__FLASH_DATA_ADDR = FLASH_DATA_ADDR;
__FLASH_DATA_SIZE = FLASH_DATA_SIZE;
__FLASH_LOG_ADDR  = FLASH_LOG_ADDR;
__FLASH_LOG_SIZE  = FLASH_LOG_SIZE;


/* System memory map */




MEMORY
{
    /* Application stored in and executes from internal flash */
    FLASH_APPL (RX) : origin = FLASH_APPL_ADDR, length = FLASH_APPL_SIZE
    /* User-defined data section in flash */
    FLASH_LOG  (RX) : origin = FLASH_LOG_ADDR , length = FLASH_LOG_SIZE
    FLASH_DATA (RX) : origin = FLASH_DATA_ADDR, length = FLASH_DATA_SIZE
    /* CCFG data */
    FLASH_CCFG (RX) : origin = FLASH_CCFG_ADDR, length = FLASH_CCFG_SIZE
    /* Application uses internal RAM for data */
    SRAM (RWX) : origin = RAM_ADDR, length = RAM_SIZE

    /* Application stored in and executes from internal flash */
    //FLASH (RX) : origin = FLASH_BASE, length = FLASH_SIZE
    /* Application uses internal RAM for data */
    //SRAM (RWX) : origin = RAM_BASE, length = RAM_SIZE
}

/* Section allocation in memory */

SECTIONS
{
/*
    .text           :   > FLASH
    .const          :   > FLASH
    .constdata      :   > FLASH
    .rodata         :   > FLASH
    .cinit          :   > FLASH
    .pinit          :   > FLASH
    .init_array     :   > FLASH
    .emb_text       :   > FLASH
    .ccfg           :   > FLASH (HIGH)
*/
    .text           :   > FLASH_APPL
    // copied next line from SDK 3.2
    .TI.ramfunc     : {} load=FLASH, run=SRAM, table(BINIT)
    .const          :   > FLASH_APPL
    .constdata      :   > FLASH_APPL
    .rodata         :   > FLASH_APPL
    .cinit          :   > FLASH_APPL
    .pinit          :   > FLASH_APPL
    .init_array     :   > FLASH_APPL
    .emb_text       :   > FLASH_APPL
    .logdata        :   > FLASH_LOG
    .flashdata      :   > FLASH_DATA
    .ccfg           :   > FLASH_CCFG (HIGH)

#ifdef REMOVED_NOT_PRESET_IN_SDK_2_20
#ifdef __TI_COMPILER_VERSION__
#if __TI_COMPILER_VERSION__ >= 15009000
    .TI.ramfunc     : {} load=FLASH, run=SRAM, table(BINIT)
#endif
#endif
#endif
    .data           :   > SRAM
    .bss            :   > SRAM
    .sysmem         :   > SRAM
    .stack          :   > SRAM (HIGH)
    .nonretenvar    :   > SRAM
}
