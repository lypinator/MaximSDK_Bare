/* ****************************************************************************
 * Copyright (C) 2019 Maxim Integrated Products, Inc., All Rights Reserved.
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
 * $Date: 2018-08-28 17:03:02 -0500 (Tue, 28 Aug 2018) $
 * $Revision: 37424 $
 *
 *************************************************************************** */
#include "adc.h"
#include "adc_regs.h"
#include "adc_reva.h"
#include "mxc_device.h"
#include "mxc_errors.h"
#include "mxc_assert.h"
#include "mxc_sys.h"
#include "mcr_regs.h"
#include "mxc_lock.h"
#include "mxc_pins.h"
#include <stdio.h>

static void initGPIOForChannel(mxc_adc_chsel_t channel)
{
    switch (channel) {
    case MXC_ADC_CH_0:
        MXC_GPIO_Config(&gpio_cfg_adc_ain0);
        break;
        
    case MXC_ADC_CH_1:
        MXC_GPIO_Config(&gpio_cfg_adc_ain1);
        break;
        
    case MXC_ADC_CH_2:
        MXC_GPIO_Config(&gpio_cfg_adc_ain2);
        break;
        
    case MXC_ADC_CH_3:
        MXC_GPIO_Config(&gpio_cfg_adc_ain3);
        break;
        
    case MXC_ADC_CH_4:
        MXC_GPIO_Config(&gpio_cfg_adc_ain4);
        break;
        
    case MXC_ADC_CH_5:
        MXC_GPIO_Config(&gpio_cfg_adc_ain5);
        break;
        
    case MXC_ADC_CH_6:
        MXC_GPIO_Config(&gpio_cfg_adc_ain6);
        break;
        
    case MXC_ADC_CH_7:
        MXC_GPIO_Config(&gpio_cfg_adc_ain7);
        break;
        
    default:
        break;
    }
}

int MXC_ADC_Init(void)
{
    MXC_SYS_Reset_Periph(MXC_SYS_RESET0_ADC);
    
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_ADC);
    
    return MXC_ADC_RevA_Init();
}

int MXC_ADC_Shutdown(void)
{
    MXC_ADC_RevA_Shutdown();
    
    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_ADC);
    
    return E_NO_ERROR;
}

int MXC_ADC_Busy(void)
{
    return MXC_ADC_RevA_Busy();
}

void MXC_ADC_EnableInt(uint32_t flags)
{
    MXC_ADC_RevA_EnableInt(flags);
}

void MXC_ADC_DisableInt(uint32_t flags)
{
    MXC_ADC_RevA_DisableInt(flags);
}

int MXC_ADC_GetFlags(void)
{
    return MXC_ADC_RevA_GetFlags();
}

void MXC_ADC_ClearFlags(uint32_t flags)
{
    MXC_ADC_RevA_ClearFlags(flags);
}

int MXC_ADC_SetConversionSpeed(uint32_t hz)
{
    //check for overflow
    MXC_ASSERT(hz < ((uint32_t)((1U << 31) - 1) / 1024));
    uint32_t adc_clock_freq = 1024 * hz;
    
    if (adc_clock_freq > MXC_ADC_MAX_CLOCK) {
        return E_BAD_PARAM;
    }
    
    uint8_t divider = PeripheralClock / adc_clock_freq;
    
    if (divider > 0xf || divider < 2) {
        return E_BAD_PARAM;
    }
    
    //disable clock
    MXC_ADC->ctrl &= ~MXC_F_ADC_CTRL_CLK_EN;
    //clear clock divisor
    MXC_GCR->pclkdiv &= (~MXC_F_GCR_PCLKDIV_ADCFRQ);
    //load in new clock divisor
    MXC_GCR->pclkdiv |= (divider << MXC_F_GCR_PCLKDIV_ADCFRQ_POS);
    
    //enable clock
    MXC_ADC_RevA_SetConversionSpeed(hz);
    
    return E_NO_ERROR;
}

int MXC_ADC_GetConversionSpeed(void)
{
    uint8_t divider = (MXC_GCR->pclkdiv & MXC_F_GCR_PCLKDIV_ADCFRQ) >> MXC_F_GCR_PCLKDIV_ADCFRQ_POS;
    return MXC_ADC_RevA_GetConversionSpeed(divider);
}

void MXC_ADC_SetDataAlignment(int msbJustify)
{
    MXC_ADC_RevA_SetDataAlignment(msbJustify);
}

void MXC_ADC_SetExtScale(mxc_adc_scale_t scale)
{
    MXC_ADC_RevA_SetExtScale(scale);
}

void MXC_ADC_EnableMonitor(mxc_adc_monitor_t monitor)
{
    MXC_ADC_RevA_EnableMonitor(monitor);
}

void MXC_ADC_DisableMonitor(mxc_adc_monitor_t monitor)
{
    MXC_ADC_RevA_DisableMonitor(monitor);
}

void MXC_ADC_SetMonitorHighThreshold(mxc_adc_monitor_t monitor, uint32_t threshold)
{
    MXC_ADC_RevA_SetMonitorHighThreshold(monitor, threshold);
}

int MXC_ADC_GetMonitorHighThreshold(mxc_adc_monitor_t monitor)
{
    return MXC_ADC_RevA_GetMonitorHighThreshold(monitor);
}

void MXC_ADC_SetMonitorLowThreshold(mxc_adc_monitor_t monitor, uint32_t threshold)
{
    MXC_ADC_RevA_SetMonitorLowThreshold(monitor, threshold);
}

int MXC_ADC_GetMonitorLowThreshold(mxc_adc_monitor_t monitor)
{
    return MXC_ADC_RevA_GetMonitorLowThreshold(monitor);
}

void MXC_ADC_SetMonitorChannel(mxc_adc_monitor_t monitor, mxc_adc_chsel_t channel)
{
    initGPIOForChannel(channel);
    
    MXC_ADC_RevA_SetMonitorChannel(monitor, channel);
}

int MXC_ADC_GetMonitorChannel(mxc_adc_monitor_t monitor)
{
    return MXC_ADC_RevA_GetMonitorChannel(monitor);
}

void MXC_ADC_EnableMonitorAsync(mxc_adc_monitor_t monitor, mxc_adc_monitor_cb_t callback)
{
    MXC_ADC_RevA_EnableMonitorAsync(monitor, callback);
}

void MXC_ADC_DisableMonitorAsync(mxc_adc_monitor_t monitor)
{
    MXC_ADC_RevA_DisableMonitorAsync(monitor);
}

int MXC_ADC_StartConversion(mxc_adc_chsel_t channel)
{
    initGPIOForChannel(channel);
    
    return MXC_ADC_RevA_StartConversion(channel);
}

int MXC_ADC_StartConversionAsync(mxc_adc_chsel_t channel, mxc_adc_complete_cb_t callback)
{
    initGPIOForChannel(channel);
    
    return MXC_ADC_RevA_StartConversionAsync(channel, callback);
}

int MXC_ADC_StartConversionDMA(mxc_adc_chsel_t channel, uint16_t* data, void (*callback)(int, int))
{
    initGPIOForChannel(channel);
    
    return MXC_ADC_RevA_StartConversionDMA(channel, data, callback);
}

int MXC_ADC_Handler(void)
{
    return MXC_ADC_RevA_Handler();
}

int MXC_ADC_Convert(mxc_adc_conversion_req_t req)
{
    initGPIOForChannel(req.channel);
    
    return MXC_ADC_RevA_Convert(req);
}

int MXC_ADC_ConvertAsync(mxc_adc_conversion_req_t req)
{
    initGPIOForChannel(req.channel);
    
    return MXC_ADC_RevA_ConvertAsync(req);
}

void MXC_ADC_Monitor(mxc_adc_monitor_req_t req)
{
    MXC_ADC_RevA_Monitor(req);
}

void MXC_ADC_MonitorAsync(mxc_adc_monitor_req_t req)
{
    MXC_ADC_RevA_MonitorAsync(req);
}

int MXC_ADC_GetData(uint16_t* outdata)
{
    return MXC_ADC_RevA_GetData(outdata);
}
