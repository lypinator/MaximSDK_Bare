/**
 * @file    main.c
 * @brief   SPI Master Demo
 * @details Shows Master loopback demo for SPI
 *          Read the printf() for instructions
 */

/*******************************************************************************
* Copyright (C) Maxim Integrated Products, Inc., All Rights Reserved.
*
* Permission is hereby granted, free of charge, to any person obtaining a
* copy of this software and associated documentation files (the "Software"),
* to deal in the Software without restriction, including without limitation
* the rights to use, copy, modify, merge, publish, distribute, sublicense,
* and/or sell copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included
* in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
* OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
* OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*
* Except as contained in this notice, the name of Maxim Integrated
* Products, Inc. shall not be used except as stated in the Maxim Integrated
* Products, Inc. Branding Policy.
*
* The mere transfer of this software does not imply any licenses
* of trade secrets, proprietary technology, copyrights, patents,
* trademarks, maskwork rights, or any other form of intellectual
* property whatsoever. Maxim Integrated Products, Inc. retains all
* ownership rights.
*
******************************************************************************/

/***** Includes *****/
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "mxc_device.h"
#include "mxc_delay.h"
#include "mxc_pins.h"
#include "nvic_table.h"
#include "uart.h"
#include "spi.h"
#include "dma.h"
#include "board.h"


/***** Preprocessors *****/
#define MASTERSYNC
// #define MASTERASYNC
// #define MASTERDMA

/***** Definitions *****/
#define DATA_LEN        100         // Words
#define DATA_VALUE      0xA5A5      // This is for master mode only...
#define VALUE           0xFFFF
#define SPI_SPEED       100000      // Bit Rate

#define SPI_INSTANCE_NUM    1

/***** Globals *****/
uint16_t rx_data[DATA_LEN];
uint16_t tx_data[DATA_LEN];
volatile int SPI_FLAG;
volatile uint8_t DMA_FLAG = 0;

/***** Functions *****/
#if defined (BOARD_FTHR_REVA)
#define SPI         MXC_SPI0
#define SPI_IRQ     SPI0_IRQn
void SPI0_IRQHandler(void)
{
    MXC_SPI_AsyncHandler(SPI);
}
#elif defined (BOARD_EVKIT_V1)
#define SPI         MXC_SPI1
#define SPI_IRQ     SPI1_IRQn
void SPI1_IRQHandler(void)
{
    MXC_SPI_AsyncHandler(SPI);
}
#else
#error "This example has been configured to work with the EV Kit or the FTHR boards."
#endif

void DMA0_IRQHandler(void)
{
    MXC_DMA_Handler();
}

void DMA1_IRQHandler(void)
{
    MXC_DMA_Handler();
    DMA_FLAG = 1;
}

void SPI_Callback(mxc_spi_req_t* req, int error)
{
    SPI_FLAG = error;
}

int main(void)
{
    int i, j, retVal;
    uint16_t temp;
    mxc_spi_req_t req;
    mxc_spi_pins_t spi_pins;
    
    printf("\n**************************** SPI MASTER TEST *************************\n");
    #if defined (BOARD_FTHR_REVA)
        printf("This example configures the SPI to send data between the MISO (P0.6) and\n");
        printf("MOSI (P0.5) pins.  Connect these two pins together.  \n\n");
    #elif defined (BOARD_EVKIT_V1)
        printf("This example configures the SPI to send data between the MISO (P0.22) and\n");
        printf("MOSI (P0.21) pins.  Connect these two pins together.  \n\n");
    #endif
    printf("Multiple word sizes (2 through 16 bits) are demonstrated.\n\n");
    
    spi_pins.clock = TRUE;
    spi_pins.miso = TRUE;
    spi_pins.mosi = TRUE;
    spi_pins.sdio2 = FALSE;
    spi_pins.sdio3 = FALSE;
    spi_pins.ss0 = TRUE;
    spi_pins.ss1 = FALSE;
    spi_pins.ss2 = FALSE;
    
#ifdef MASTERSYNC
    printf("Performing blocking (synchronous) transactions...\n");
#endif
#ifdef MASTERASYNC
    printf("Performing non-blocking (asynchronous) transactions...\n");
#endif
#ifdef MASTERDMA
    printf("Performing transactions with DMA...\n");
#endif
    
    for (i = 2; i < 17; i++) {
        // Sending out 2 to 16 bits

        // The hardware doesn't support 9-bit wide characters at high speeds.
        if(i == 9) {
            printf("Hardware does not support 9-bit wide characters.\n");
            continue;
        }
        
        for (j = 0; j < DATA_LEN; j++) {
            tx_data[j] = DATA_VALUE;
        }
        
        // Configure the peripheral
        if (MXC_SPI_Init(SPI, 1, 0, 1, 0, SPI_SPEED, spi_pins) != E_NO_ERROR) {
            printf("\nSPI INITIALIZATION ERROR\n");
            
            while (1) {}
        }
        
        memset(rx_data, 0x0, DATA_LEN * sizeof(uint16_t));
        
        //SPI Request
        req.spi = SPI;
        req.txData = (uint8_t*) tx_data;
        req.rxData = (uint8_t*) rx_data;
        req.txLen = DATA_LEN;
        req.rxLen = DATA_LEN;
        req.ssIdx = 0;
        req.ssDeassert = 1;
        req.txCnt = 0;
        req.rxCnt = 0;
        req.completeCB = (spi_complete_cb_t) SPI_Callback;
        SPI_FLAG = 1;
        
        retVal = MXC_SPI_SetDataSize(SPI, i);
        
        if (retVal != E_NO_ERROR) {
            printf("\nSPI SET DATASIZE ERROR: %d\n", retVal);
            
            while (1) {}
        }
        
        retVal = MXC_SPI_SetWidth(SPI, SPI_WIDTH_STANDARD);
        
        if (retVal != E_NO_ERROR) {
            printf("\nSPI SET WIDTH ERROR: %d\n", retVal);
            
            while (1) {}
        }
        
#ifdef MASTERSYNC
        MXC_SPI_MasterTransaction(&req);
#endif
        
#ifdef MASTERASYNC
        NVIC_EnableIRQ(SPI_IRQ);
        MXC_SPI_MasterTransactionAsync(&req);
        
        while (SPI_FLAG == 1);
        
#endif
        
#ifdef MASTERDMA
        MXC_DMA_ReleaseChannel(0);
        MXC_DMA_ReleaseChannel(1);
        
        NVIC_EnableIRQ(DMA0_IRQn);
        NVIC_EnableIRQ(DMA1_IRQn);
        MXC_SPI_MasterTransactionDMA(&req);
        
        while (DMA_FLAG == 0);
        
        DMA_FLAG = 0;
#endif
        
        uint8_t bits = MXC_SPI_GetDataSize(SPI);
        
        for (j = 0; j < DATA_LEN; j++) {
            if (bits <= 8) {
                if (j < (DATA_LEN / 2)) {
                    temp = VALUE >> (16 - bits);
                    temp = (temp << 8) | temp;
                    temp &= DATA_VALUE;
                    tx_data[j] = temp;
                }
                else if (j == (DATA_LEN / 2) && DATA_LEN % 2 == 1) {
                    temp = VALUE >> (16 - bits);
                    temp &= DATA_VALUE;
                    tx_data[j] = temp ;
                }
                else {
                    tx_data[j] = 0x0000;
                }
            }
            else {
                temp = VALUE >> (16 - bits);
                temp &= DATA_VALUE;
                tx_data[j] = temp;
            }
        }
        
        // Compare Sent data vs Received data
        // Printf needs the Uart turned on since they share the same pins
        if (memcmp(rx_data, tx_data, sizeof(tx_data)) != 0) {
            printf("\n-->%2d Bits Transaction Failed\n", i);
            
            while (1) {}
        }
        else {
            printf("-->%2d Bits Transaction Successful\n", i);
        }
        
        retVal = MXC_SPI_Shutdown(SPI);
        
        if (retVal != E_NO_ERROR) {
            printf("\n-->SPI SHUTDOWN ERROR: %d\n", retVal);
            
            while (1) {}
        }
    }
    
    printf("\nExample Complete.\n");
    return E_NO_ERROR;
}
