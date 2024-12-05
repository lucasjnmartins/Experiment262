#include <stdio.h>
#include <stdbool.h>
#include <sys/_intsup.h>
#include <unistd.h>
#include "dshot/dshot.h"
#include "soc/adc_channel.h"
#include "soc/gpio_num.h"
#include "gpio/gpio_config.h"
#include "esp_adc/adc_continuous.h"
#include "esp_bit_defs.h"
#include "hal/adc_types.h"
#include "adc/adc.h"

#define DSHOT_MOTOR_R 13

#define DSHOT_MOTOR_L 14
#define MOTOR_MAX_SPEED 140
#define MOTOR_BASE_SPEED 70
#define GPIO_MOSFET 23
#define GPIO_LED1 17
#define GPIO_LED2 5

extern "C" void app_main(void)
{
	printf("Running Experiment262");
	Gpio Mosfet(static_cast<gpio_num_t>(GPIO_MOSFET));
	Gpio Led1(static_cast<gpio_num_t>(GPIO_LED1));
	Gpio Led2(static_cast<gpio_num_t>(GPIO_LED2));
	Dshot MotorR(static_cast<gpio_num_t>(DSHOT_MOTOR_R), true, MOTOR_MAX_SPEED);
	Dshot MotorL(static_cast<gpio_num_t>(DSHOT_MOTOR_L), false, MOTOR_MAX_SPEED);
	adc_channel_t adc1[6] = {ADC_CHANNEL_6, ADC_CHANNEL_7, ADC_CHANNEL_3, ADC_CHANNEL_4, ADC_CHANNEL_0, ADC_CHANNEL_5};
	adc_channel_t adc2[3] = {ADC_CHANNEL_7, ADC_CHANNEL_8, ADC_CHANNEL_9};
	
	float result[9];
	uint8_t size = 9;
	float error = 0;
	Adc Adc(adc1, adc2, 1);
	
	Led1.ConfigureGPIO();
	Led2.ConfigureGPIO();	
	Led1.UpdateGPIO(true);
	Led2.UpdateGPIO(true);
	
	MotorR.ConfigureDshot();
	MotorL.ConfigureDshot();
	Mosfet.ConfigureGPIO();
	Mosfet.UpdateGPIO(true);	
	
	Adc.ConfigureAdc();
	
	for(int i = 0; i < 700; i++) {
		for(int j = 0; j < 9; j++) {
			MotorR.UpdateThrottle(MOTOR_BASE_SPEED);
		   	MotorL.UpdateThrottle(-MOTOR_BASE_SPEED);
		   	Adc.Calibration(j);
	   	}
	}
	MotorR.UpdateThrottle(0);
	MotorL.UpdateThrottle(0);
	vTaskDelay(pdMS_TO_TICKS(5000));
    while (true) {      
       for(uint8_t i=0; i < size; i++) {
		   result[i] = Adc.ReadAdc(i);
	   }
	   if (true) {
	   		error = (result[1] - result[0]) + 2*(result[3] - result[2]) + 3*(result[5] - result[4]);
	   } else {
		   	error = (result[0] - result[1]) + 2*(result[2] - result[3]) + 3*(result[4] - result[5]);
	   }
	   
	   printf("\n %f", error);
	   
	   MotorR.UpdateThrottle(MOTOR_BASE_SPEED + 1.5*error);
	   MotorL.UpdateThrottle(MOTOR_BASE_SPEED - 1.5*error);
	   vTaskDelay(pdMS_TO_TICKS(1));
	   if(result[0] > 50) {
		   Led1.UpdateGPIO(true);
			Led2.UpdateGPIO(false);
	   } else {
		   Led1.UpdateGPIO(false);
			Led2.UpdateGPIO(true);
	   }
    }
}
