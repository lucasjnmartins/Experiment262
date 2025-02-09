#include <stdio.h>
#include <stdbool.h>
#include <sys/_intsup.h>
#include <sys/_stdint.h>
#include <unistd.h>
#include "arch/sys_arch.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "dshot/dshot.h"
#include "soc/adc_channel.h"
#include "soc/gpio_num.h"
#include "gpio/gpio_config.h"
#include "esp_adc/adc_continuous.h"
#include "esp_bit_defs.h"
#include "hal/adc_types.h"
#include "adc/adc.h"
#include "encoder/encoder.h"

#define DSHOT_MOTOR_R 13

#define DSHOT_MOTOR_L 14
#define MOTOR_MAX_SPEED 130
#define MOTOR_BASE_SPEED 70
#define GPIO_MOSFET 23
#define GPIO_LED1 17
#define GPIO_LED2 5
#define GPIO_ENCR_CHA 18
#define GPIO_ENCR_CHB 19
#define GPIO_ENCL_CHA 16
#define GPIO_ENCL_CHB 4

// 0 - White line on a black surface
// 1 - Black line on a white surface
#define TRACE_COLOR	0
			 
// Classes presenting the project components
Gpio mosfet(static_cast<gpio_num_t>(GPIO_MOSFET));
Gpio ledWhite(static_cast<gpio_num_t>(GPIO_LED1));
Gpio ledYellow(static_cast<gpio_num_t>(GPIO_LED2));
Dshot motorL(static_cast<gpio_num_t>(DSHOT_MOTOR_L), false, MOTOR_MAX_SPEED);
Dshot motorR(static_cast<gpio_num_t>(DSHOT_MOTOR_R), true, MOTOR_MAX_SPEED);
adc_channel_t adc1[6] = {ADC_CHANNEL_6, ADC_CHANNEL_7, ADC_CHANNEL_3, ADC_CHANNEL_4, ADC_CHANNEL_0, ADC_CHANNEL_5};
adc_channel_t adc2[3] = {ADC_CHANNEL_7, ADC_CHANNEL_8, ADC_CHANNEL_9};
Adc adc(adc1, adc2, TRACE_COLOR);
Encoder encoderL(GPIO_ENCL_CHA, GPIO_ENCL_CHB, 0);
Encoder encoderR(GPIO_ENCR_CHA, GPIO_ENCR_CHB, 0);

float adcSensors[9] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
const float kp = 1.6, kd = 1.0;
uint8_t aceleration[28] = 	{5,5,15,15,10,8,5,
							5,15,5,5,5,5,5,
							5, 5 ,5 ,40,20,10,5,
							5,5,5, 5,5,5,5};

// Handles for the freeRTOS tasks
TaskHandle_t xHandleCalibration = NULL;
TaskHandle_t xHandleFollowLine = NULL;
TaskHandle_t xHandleReadSensor = NULL;
TaskHandle_t xHandleCountCheckpoint = NULL;
TaskHandle_t xHandleReadEncoder = NULL;

static int countR = 0, countL= 0;

void readEncoder(void *parameters){
	printf("readEncoder");
	int leftEncoder = 0, rightEncoder = 0;
	for(;;){
		
		encoderL.ReadEncoder(&leftEncoder);
		encoderR.ReadEncoder(&rightEncoder);
		printf("%d - %d", leftEncoder, rightEncoder);
		printf("\n");
		vTaskDelay(pdMS_TO_TICKS(10));
	}
}
	
void countCheckpoint(void *parameters){
	static bool stateL = false, stateR = false;
	
	printf("countCheckpoint");
	
	for (;;) {
		
		//Left
		adcSensors[7] = adc.ReadAdc(7);
		if(adcSensors[7] > 2.5 && stateL == false){
			
		
			stateL = true;
			countL++;
		} 
		if(adcSensors[7] < 2.5 && stateL == true){
			stateL = false;
			
		}
		
		//Right
		adcSensors[8] = adc.ReadAdc(8);
		if(adcSensors[8] > 2.5 && stateR == false){
			ledWhite.UpdateGPIO(true);
			ledYellow.UpdateGPIO(true);
			stateR = true;
			countR++;
		} 
		if(adcSensors[8] < 2.5 && stateR == true){
			ledWhite.UpdateGPIO(false);
			ledYellow.UpdateGPIO(false);
			stateR = false;
		}
		vTaskDelay(pdMS_TO_TICKS(10));
	}
}

void followLine(void *parameters) {
	static float error = 0, lastError = 0, sensorSum = 0 ;
	static int acel = 0;
	mosfet.UpdateGPIO(true);
	for(;;) {  
		for(uint8_t i=0; i<6; i++) {
			adcSensors[i] = adc.ReadAdc(i);
		}
		lastError = error;
		sensorSum = adcSensors[0] + adcSensors[1] + adcSensors[2] + adcSensors[3]+ adcSensors[4] + adcSensors[5];
		error = (adcSensors[1] - adcSensors[0]) + 2*(adcSensors[3] - adcSensors[2]) + 3*(adcSensors[5] - adcSensors[4]);
	 
	 	if (error > -5.0 && error < 5.0 && acel < aceleration[countL]) {
			 acel++;
		 } else {
			 acel--;
		 }
		 if (acel > aceleration[countL]) {
			  acel = aceleration[countL];
		 }
		 if (acel < 0) {
			 acel = 0;
		 }
	    float controllerResult = kp*error + kd*(error - lastError);
	    if(sensorSum < 5 || countR > 5){
			motorR.UpdateThrottle(0);
	    	motorL.UpdateThrottle(0);
		} else {
			motorR.UpdateThrottle(MOTOR_BASE_SPEED + controllerResult + acel);
	    	motorL.UpdateThrottle(MOTOR_BASE_SPEED - controllerResult + acel);
		}
	  
	    vTaskDelay(pdMS_TO_TICKS(2));
	}
}

void calibration(void *parameters) {
	for(;;) {
		mosfet.UpdateGPIO(true);
		ledWhite.UpdateGPIO(true);
		int speed = MOTOR_BASE_SPEED + 15;
		for(int i = 0; i < 1000; i++) {
			for(int j = 0; j < 9; j++) {
				motorR.UpdateThrottle(speed);
			   	motorL.UpdateThrottle(-speed);
			   	adc.Calibration(j);
		   	}
		}
		motorR.UpdateThrottle(0);
		motorL.UpdateThrottle(0);
		ledWhite.UpdateGPIO(false);
		
		vTaskDelay(pdMS_TO_TICKS(5000));
		xTaskCreatePinnedToCore(followLine, "FollowLine", 4096, NULL, 2, &xHandleFollowLine, 0);
		//xTaskCreatePinnedToCore(countCheckpoint, "CountCheckpoints", 4096, NULL, 2, &xHandleCountCheckpoint, 1);
		xTaskCreatePinnedToCore(readEncoder, "ReadEncoder", 4096, NULL, 2, &xHandleReadEncoder, 1);

		vTaskSuspend(xHandleCalibration);
		vTaskDelete(NULL);
	}
}



extern "C" void app_main(void)
{
	printf("Running Experiment262");
	
	ledWhite.ConfigureGPIO();
	ledYellow.ConfigureGPIO();	
	motorL.ConfigureDshot();	
	motorR.ConfigureDshot();	
	vTaskDelay(pdMS_TO_TICKS(5000));
	mosfet.ConfigureGPIO();
	adc.ConfigureAdc();
	encoderL.ConfigureEncoder();
	encoderR.ConfigureEncoder();	
	mosfet.UpdateGPIO(true);
	
	xTaskCreatePinnedToCore(calibration, "Calibration", 4096, NULL, 24, &xHandleCalibration, 0);
}
