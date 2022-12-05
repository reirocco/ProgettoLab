/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @author			: Nori Rocco (S1100082)
  * @file           : currentsensorfcn.h
  * @brief          : Header for correct current sensor uses .
  *              	This file contains function for the calibration and use of customized current sensor
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CURRENT_SENSOR_FCN
#define CURRENT_SENSOR_FCN
#endif

/* Defines -------------------------------------------------------------------*/
#define VREF 2.5				// Vref of the current sensor
#define READING_ADC_TIME 0.00416 	// MILLISECONDI  utilizzati dal nucleo PER effettuare UNA RILEVAZIONE DEL ADC MISURATI CON L'OSCILLOSCOPIO
#define CALIBRATION_TIME 5000	// calibration time in milliseconds
#define ADC_TIMEOUT 100
#define FILTER 8
#define ADC_RESOLUTION 4096		// adc resolution for 12 bit
double realVRef;
uint32_t bMedia;
/* Functions -----------------------------------------------------------------*/


int calibration(int adcStartAddr, int adcPollForConversionAddr, int adcGetValue, int hadcAddr){
	// pointer to a adc start function
	void (*adcStartFcn)(int) = adcStartAddr;
	// pointer to a adc poll function
	HAL_StatusTypeDef (*adcPollForConversionFcn)(int, int) = adcPollForConversionAddr;
	// pointer to a adc GetValue function
	uint32_t (*adcGetValueFcn)(int) = adcGetValue;

	/* loop for at least 5 seconds and average the values to calculate the offset
	 * 5 seconds --> 5000ms / READING_ADC_TIME = cycle to do
	 */
	uint32_t bSum = 0;
	double sum;
	double average;
	int range = (int) CALIBRATION_TIME / READING_ADC_TIME;

	double rilevazione;
	for(int i = 0; i <= (int)range; i++){
		adcStartFcn(hadcAddr);
		adcPollForConversionFcn(hadcAddr,ADC_TIMEOUT);
		rilevazione = adcGetValueFcn(hadcAddr);
		bSum = bSum + rilevazione;
	}
	sum = bSum * 3.3 /(ADC_RESOLUTION-1);	//convert binary sum to numeric value
	average = sum / range;
	printf("%f\r\n", average);
	realVRef = VREF + (VREF - average);	// calculating voltage offset
}


double getFilteredValue(int adcStartAddr, int adcPollForConversionAddr, int adcGetValue, int hadcAddr){
	// pointer to a adc start function
	void (*adcStartFcn)(int) = adcStartAddr;
	// pointer to a adc poll function
	HAL_StatusTypeDef (*adcPollForConversionFcn)(int, int) = adcPollForConversionAddr;
	// pointer to a adc GetValue function
	uint32_t (*adcGetValueFcn)(int) = adcGetValue;

	uint32_t CountValue;
	double delta;
	double voltMedia;
	double current;

	adcStartFcn(hadcAddr);
	adcPollForConversionFcn(hadcAddr,ADC_TIMEOUT);
	CountValue = adcGetValueFcn(hadcAddr);

	if (bMedia > CountValue){
		delta = ((double)(bMedia - CountValue)) / FILTER;
		bMedia -= delta;
	}else if (bMedia < CountValue){
		delta = ((double)(CountValue - bMedia )) / FILTER;
		bMedia += delta;
	}


	voltMedia = bMedia * 3.3 /(ADC_RESOLUTION-1);
	current =  (realVRef - voltMedia) / 0.4;
	return current;
}
