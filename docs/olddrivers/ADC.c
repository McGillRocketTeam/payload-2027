/*
 * ADC.c
 *
 *  Created on: Jun 8, 2025
 *      Author: akash
 */

#include "ADC.h"
#include "accelerometer.h"
#include "enabled.h"
#include "serial_monitor.h"

/* 
 * To get more accurate readings, maybe consider measuring the STM32's internal reference voltage `Vrefint` channel.
 * Resource: https://community.st.com/t5/stm32-mcus/how-to-use-the-stm32-adc-s-internal-reference-voltage/ta-p/621425
 */

bool PL_ADC_Init(
	PL_ADC_Handler *adc,
	ADC_HandleTypeDef *hadc1,
	ADC_HandleTypeDef *hadc2,
	ADC_HandleTypeDef *hadc3,
	TIM_HandleTypeDef *htim,
	volatile uint16_t accelerometer_buffer[ACCELEROMETER_SAMPLE_SIZE_TRIPLE])
{
    // Initialize struct
    adc->hadc1 = hadc1;
    adc->hadc2 = hadc2;
    adc->hadc3 = hadc3;
    adc->htim = htim;
    //adc->vbat_sample = 0;
    //adc->current_sample = 0;
    
    // Start ADCs
    // Starting ADC1 in triple synchronous multimode with DMA takes the values from the other two ADCs
    HAL_StatusTypeDef adc2_status = HAL_ADC_Start(hadc2);
    HAL_StatusTypeDef adc3_status = HAL_ADC_Start(hadc3);
    HAL_StatusTypeDef adc1_status = HAL_ADCEx_MultiModeStart_DMA(hadc1, (uint32_t *) accelerometer_buffer, ACCELEROMETER_SAMPLE_SIZE_TRIPLE);

    // NOTE: the sample timer (TIM8) is NOT started here. It is owned and started by
    // the accelerometer module (PL_Accelerometer_Start -> HAL_TIM_Base_Start). The ADCs
    // are hardware-triggered by its TRGO output and the data path is driven by the DMA
    // conversion callbacks, so TIM8's update interrupt is not needed. Starting it here
    // too made HAL_TIM_Base_Start_IT return HAL_ERROR (timer already running).
    (void) htim;

    bool ok = adc1_status == HAL_OK && adc2_status == HAL_OK && adc3_status == HAL_OK;

    // On failure, report the individual return statuses so the failing call can be
    // pinpointed. Note: HAL_ADC_Start always returns HAL_OK, and a MultiModeStart_DMA
    // failure comes from the DMA layer (HAL_DMA_Start_IT) and does NOT set the ADC
    // error code -- so the DMA handle state/error is what actually explains adc1_status.
    // Status codes: 0=OK, 1=ERROR, 2=BUSY, 3=TIMEOUT.
    if (!ok) {
        PL_Log(LOG_ADC, LOG_NONE, LOG_ERROR,
               "start status: adc1=%d adc2=%d adc3=%d | dma.State=0x%02lX dma.Err=0x%lX",
               adc1_status, adc2_status, adc3_status,
               (uint32_t) hadc1->DMA_Handle->State,
               HAL_DMA_GetError(hadc1->DMA_Handle));
    }

    return ok;
}

bool PL_ADC_Deinit(PL_ADC_Handler *adc)
{
	HAL_StatusTypeDef adc1_status = HAL_ADCEx_MultiModeStop_DMA(adc->hadc1);
    HAL_StatusTypeDef adc2_status = HAL_ADC_Stop(adc->hadc2);
    HAL_StatusTypeDef adc3_status = HAL_ADC_Stop(adc->hadc3);
    // The sample timer (TIM8) is owned by the accelerometer module; stopping it is
    // PL_Accelerometer_Stop's responsibility, not the ADC's (mirror of PL_ADC_Init).
    return adc1_status == HAL_OK && adc2_status == HAL_OK && adc3_status == HAL_OK;
}
