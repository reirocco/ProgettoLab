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
#define CALIBRATION_TIME 1000	// calibration time in milliseconds
#define ADC_TIMEOUT 100
#define FILTER 15
#define ADC_RESOLUTION 4096		// adc resolution for 12 bit
#define TORQUECONSTANT 0,1515573181818224	// N/m torque constant --> see https://www.pololu.com/product/4751/specs#note3
double realVRef;
uint32_t bMedia = 2.5;


/* Proto ---------------------------------------------------------------------*/
double getFilteredValue(int adcStartAddr, int adcPollForConversionAddr, int adcGetValue, int hadcAddr);
int calibration(int adcStartAddr, int adcPollForConversionAddr, int adcGetValue, int hadcAddr);
double getCurrentValue(int adcStartAddr, int adcPollForConversionAddr, int adcGetValue, int hadcAddr);

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
	double sum = 0;
	double average;
	//int range = (int) CALIBRATION_TIME / READING_ADC_TIME;
	int range = 1000;

	double rilevazione;

	for(int i = 0; i < 20; i++){
			getFilteredValue(adcStartAddr,adcPollForConversionAddr,adcGetValue,hadcAddr );
		}

	for(int i = 0; i < (int)range; i++){
		sum = sum + getFilteredValue(adcStartAddr,adcPollForConversionAddr,adcGetValue,hadcAddr );
		//HAL_Delay(15);
	}
	average = sum / (range);
	realVRef = average;	// calculating voltage offset
	bMedia = average;
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
	//printf("%f\n",CountValue *3.3 /(ADC_RESOLUTION-1));

	if (bMedia > CountValue){
		delta = ((double)(bMedia - CountValue)) / FILTER;
		bMedia -= delta;
	}else if (bMedia < CountValue){
		delta = ((double)(CountValue - bMedia )) / FILTER;
		bMedia += delta;
	}


	voltMedia = bMedia * 3.3 /(ADC_RESOLUTION-1);
	return voltMedia;
}


double processCurrent(double voltMedia){
	double current =  (realVRef - voltMedia) / 0.4;
	//printf("%f\n",realVRef);
	//printf("%f\n",voltMedia);
	return current;
}

double getCurrentValue(int adcStartAddr, int adcPollForConversionAddr, int adcGetValue, int hadcAddr){
	return processCurrent(getFilteredValue(adcStartAddr,adcPollForConversionAddr,adcGetValue,hadcAddr ));
}


double getTorque(double current){
	return (TORQUECONSTANT * current);
}
