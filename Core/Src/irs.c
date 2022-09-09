/*
 * irs.c
 */

#include "main.h"
#include "irs.h"
#include "delay.h"

// This is the buffer that will get filled up with all the measurements
uint16_t adc_buf[NUM_SAMPLES];
// "boolean" variable to keep say when the ADC has finished filling up the buffer
uint8_t complete = 0;

uint32_t usDelay = 100;

uint16_t readIR(IR ir)
{
	uint16_t readValue;

	switch(ir)
	    {
	        case IR_FORWARD_LEFT:
	        	HAL_GPIO_WritePin(ForwardLeftEmitter_GPIO_Port, ForwardLeftEmitter_Pin, GPIO_PIN_SET);
	        	delayMicroseconds(usDelay);
	        	readValue = analogRead(ir);
	        	HAL_GPIO_WritePin(ForwardLeftEmitter_GPIO_Port, ForwardLeftEmitter_Pin, GPIO_PIN_RESET);
	        	return readValue;
	            break;
	        case IR_LEFT:
	        	HAL_GPIO_WritePin(LeftEmitter_GPIO_Port, LeftEmitter_Pin, GPIO_PIN_SET);
	        	delayMicroseconds(usDelay);
	        	readValue = analogRead(ir);
	        	HAL_GPIO_WritePin(LeftEmitter_GPIO_Port, LeftEmitter_Pin, GPIO_PIN_RESET);
	        	return readValue;
	            break;
	        case IR_RIGHT:
	        	HAL_GPIO_WritePin(RightEmitter_GPIO_Port, RightEmitter_Pin, GPIO_PIN_SET);
	        	delayMicroseconds(usDelay);
	        	readValue = analogRead(ir);
	        	HAL_GPIO_WritePin(RightEmitter_GPIO_Port, RightEmitter_Pin, GPIO_PIN_RESET);
	        	return readValue;
	            break;
	        case IR_FORWARD_RIGHT:
	        	HAL_GPIO_WritePin(ForwardRightEmitter_GPIO_Port, ForwardRightEmitter_Pin, GPIO_PIN_SET);
	        	delayMicroseconds(usDelay);
	        	readValue = analogRead(ir);
	        	HAL_GPIO_WritePin(ForwardRightEmitter_GPIO_Port, ForwardRightEmitter_Pin, GPIO_PIN_RESET);
	        	return readValue;
	            break;
	        default:
	            return 0;
	    }
}

uint16_t readForwardLeftIR(void)
{
	return readIR(IR_FORWARD_LEFT);
}

uint16_t readLeftIR(void)
{
	return readIR(IR_LEFT);
}

uint16_t readRightIR(void)
{
	return readIR(IR_RIGHT);
}

uint16_t readForwardRightIR(void)
{
	return readIR(IR_FORWARD_RIGHT);
}

uint16_t analogRead(IR ir)
{
    ADC_ChannelConfTypeDef sConfig = {0}; //this initializes the IR ADC [Analog to Digital Converter]
    ADC_HandleTypeDef *hadc1_ptr = Get_HADC1_Ptr(); //this is a pointer to your hal_adc
    //this pointer will also be used to read the analog value, val = HAL_ADC_GetValue(hadc1_ptr);

    //this picks the IR direction to choose the right ADC.
    switch(ir)
    {
        case IR_FORWARD_LEFT:
            sConfig.Channel = ADC_CHANNEL_6;
            break;
        case IR_LEFT:
            sConfig.Channel = ADC_CHANNEL_5;
            break;
        case IR_RIGHT:
            sConfig.Channel = ADC_CHANNEL_11;
            break;
        case IR_FORWARD_RIGHT:
            sConfig.Channel = ADC_CHANNEL_10;
            break;
        default:
            return 0;
    }

    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;

    // make sure everything was set up correctly
    if (HAL_ADC_ConfigChannel(hadc1_ptr, &sConfig) != HAL_OK)
    {
        return 0;
    }

    complete = 0;

    // start filling up the ADC buffer
    HAL_ADC_Start_DMA(hadc1_ptr, (uint32_t*)adc_buf, NUM_SAMPLES);

    // wait for the buffer to become full
    while (complete == 0)
    {
        continue;
    }

    uint32_t sum = 0;
    // calculate the sum of the measurements in order to calculate the average
    uint16_t measurement = 0;
    while(measurement < NUM_SAMPLES) //this takes multiple measurements
    {
        sum += adc_buf[measurement];
        ++measurement;
    }

    return sum/NUM_SAMPLES;
}

/*
 This function is called when the ADC buffer is filled
 It stops the ADC and changes our "complete" variable to be "true"
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    // stop the ADC
    HAL_ADC_Stop_DMA(hadc);
    complete = 1;
}
