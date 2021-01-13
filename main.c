  /******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  * Member 1: Jonathan Chua En Zhe (A0148165B)
  * Member 2: Tan Yan Quan (A0150128R)
  * (c) EE2028 Teaching Team
  ******************************************************************************/


/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_accelero.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_magneto.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_psensor.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_tsensor.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_hsensor.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_gyro.h"
#include "stdio.h"
#include "string.h"
#include "math.h"
#include "wifi.h"
#include <stdlib.h>

#define MAX_LENGTH 400  // adjust it depending on the max size of the packet you expect to send or receive
#define WIFI_READ_TIMEOUT 10000
#define WIFI_WRITE_TIMEOUT 10000

/* initialisation functions */
static void MX_GPIO_Init(void);
static void Accelero_Int_Init(void);
static void UART1_Init(void);

UART_HandleTypeDef huart1;

//extern void initialise_monitor_handles(void);	// for semi-hosting support (printf)

/* global variables for interrupts */
volatile unsigned int last_button_press_tick = 0;
volatile unsigned int mode_changed_flag = 0; // this is changed in button press interrupt
volatile unsigned int operation_mode = 0; // 0 = healthy, 1 = intensive care
volatile unsigned int accelero_flag = 0;

/* wifi */
const char* WiFi_SSID = "Jon Chua";               // Replacce mySSID with WiFi SSID for your router / Hotspot
const char* WiFi_password = "12345678";   // Replace myPassword with WiFi password for your router / Hotspot
const WIFI_Ecn_t WiFi_security = WIFI_ECN_WPA2_PSK; // WiFi security your router / Hotspot. No need to change it unless you use something other than WPA2 PSK
const uint16_t SOURCE_PORT = 1234;  // source port, which can be almost any 16 bit number
uint8_t ipaddr[4] = {192, 168, 0, 103};

const char* SERVER_NAME = "blynk-cloud.com";    // domain name of the IoT server used
const uint16_t DEST_PORT = 80;

SPI_HandleTypeDef hspi3;

int main(void)
{
	//initialise_monitor_handles(); // for semi-hosting support (printf)

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Initialise LED2 GPIO*/
	MX_GPIO_Init();

	/* Initialise Accelerometer + Button Interrupt */
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
	Accelero_Int_Init();

	NVIC_SetPriority(SysTick_IRQn, 0x00);
	NVIC_SetPriority(EXTI15_10_IRQn, 0x40);

	/* Peripheral initializations using BSP functions */
	BSP_ACCELERO_Init();
	BSP_MAGNETO_Init();
	BSP_PSENSOR_Init();
	BSP_TSENSOR_Init();
	BSP_HSENSOR_Init();
	BSP_GYRO_Init();

	/* Initialise Telemetry */
	UART1_Init();

	/* wifi */
	uint8_t req[MAX_LENGTH];  // request packet
	uint8_t resp[MAX_LENGTH]; // response packet
	uint16_t Datalen;
	WIFI_Status_t WiFi_Stat; // WiFi status. Should remain WIFI_STATUS_OK if everything goes well

	WiFi_Stat = WIFI_Init();                      // if it gets stuck here, you likely did not include EXTI1_IRQHandler() in stm32l4xx_it.c as mentioned above
	WiFi_Stat &= WIFI_Connect(WiFi_SSID, WiFi_password, WiFi_security); // joining a WiFi network takes several seconds. Don't be too quick to judge that your program has 'hung' :)
	if(WiFi_Stat!=WIFI_STATUS_OK) while(1);                   // halt computations if a WiFi connection could not be established.

	WiFi_Stat = WIFI_GetHostAddress(SERVER_NAME, ipaddr); // DNS lookup to find the ip address, if using a connection to an IoT server

	/* ticks storage */
	unsigned int last_led_toggle_tick = 0;
	unsigned int last_1s_tick = 0;
	unsigned int last_10s_tick = 0;

	/* flags storage */
	unsigned int blink_led = 0; // this is 1 when led should be blinking
	unsigned int warning_flags[5] = {0, 0, 0, 0, 0};

	/* threshold settings */
	float acc_threshold = 0.5;
	float temp_threshold = 32.0;
	float gyro_threshold = 360.0;
	float magneto_threshold = 0.4;
	float humidity_threshold = 50;
	float baro_threshold = 1011.0;

	/* telemetry message */
	unsigned int msg_count = 0;
	char message_print[50];

	//printf("Entering Healthy Mode.\r\n");
	sprintf(message_print, "%s", "Entering Healthy Mode.\r\n");
	HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
	WiFi_Stat = WIFI_OpenClientConnection(1, WIFI_TCP_PROTOCOL, "conn", ipaddr, DEST_PORT, SOURCE_PORT);
	sprintf((char*)req, "GET /rvXK2x3GtPfvJnaNbHA4ln50v2unophS/update/V2?value=%d HTTP/1.1\r\nHost: blynk-cloud.com\r\n\r\n", operation_mode);
	WiFi_Stat = WIFI_SendData(1, req, (uint16_t)strlen((char*)req), &Datalen, WIFI_WRITE_TIMEOUT);
	WiFi_Stat = WIFI_CloseClientConnection(1);

	while(1){
		/* sensor readings */
		float accel_data[3], temp_data, gyro_data, magneto_data[3], humidity_data, baro_data;
		float accel_mag, magneto_mag;

		unsigned int current_tick = HAL_GetTick();

		if(HAL_GetTick() - last_led_toggle_tick > 250 && blink_led){
			last_led_toggle_tick = current_tick;
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
		}

		if(current_tick - last_1s_tick > 1000){
			last_1s_tick = current_tick;

			// read all sensors for healthy mode

			/* Read Accelerometer */
			int16_t accel_data_i16[3] = { 0 };			// array to store the x, y and z readings.
			BSP_ACCELERO_AccGetXYZ(accel_data_i16);		// read accelerometer
														// the function above returns 16 bit integers which are 100 * acceleration_in_m/s2. Converting to float to print the actual acceleration.
			accel_data[0] = (float)accel_data_i16[0] / 1000.0f;
			accel_data[1] = (float)accel_data_i16[1] / 1000.0f;
			accel_data[2] = (float)accel_data_i16[2] / 1000.0f;

			/* Read Temperature */
			temp_data = BSP_TSENSOR_ReadTemp();

			// read all sensors for ICU mode
			if(operation_mode == 1){
				/* Read Gyroscope */
				float gyro_raw[3];
				BSP_GYRO_GetXYZ(gyro_raw);
				gyro_raw[0] = gyro_raw[0] / 1000.0f - 0.14; // offset determined empirically
				gyro_raw[1] = gyro_raw[1] / 1000.0f + 0.35; // convert mdps to dps
				gyro_raw[2] = gyro_raw[2] / 1000.0f - 1.54;
				//printf("GYRO_%4.2f_%4.2f_%4.2f \r\n", gyro_raw[0], gyro_raw[1], gyro_raw[2]);
				gyro_data = sqrt(gyro_raw[0] * gyro_raw[0] + gyro_raw[1] * gyro_raw[1] + gyro_raw[2] * gyro_raw[2]);

				/* Read Magnetometer */
				int16_t magneto_data_i16[3] = { 0 };
				BSP_MAGNETO_GetXYZ(magneto_data_i16);
				magneto_data[0] = (float)magneto_data_i16[0] / 6842.0f;
				magneto_data[1] = (float)magneto_data_i16[1] / 6842.0f;
				magneto_data[2] = (float)magneto_data_i16[2] / 6842.0f;

				/* Read Humidity */
				humidity_data = BSP_HSENSOR_ReadHumidity();

				/* Read Barometer */
				baro_data = BSP_PSENSOR_ReadPressure();
			}

			// check all sensors for healthy mode
			accel_mag = sqrt(accel_data[0] * accel_data[0] + accel_data[1] * accel_data[1] + accel_data[2] * accel_data[2]);
			if(accel_mag < acc_threshold && warning_flags[0] == 0){
				//printf("Falling is detected \r\n");
				sprintf(message_print, "%s", "Falling is detected \r\n");
				HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
				blink_led = 1;
				warning_flags[0] = 1;
			}

			if(temp_data > temp_threshold && warning_flags[1] == 0){
				//printf("Fever is detected \r\n");
				sprintf(message_print, "%s", "Fever is detected \r\n");
				HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
				blink_led = 1;
				warning_flags[1] = 1;
			}

			// check all sensors for ICU mode
			if(operation_mode == 1){
				if(gyro_data > gyro_threshold && warning_flags[2] == 0){
					//printf("Patient in pain! \r\n");
					sprintf(message_print, "%s", "Patient in pain! \r\n");
					HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
					blink_led = 1;
					warning_flags[2] = 1;
				}

				magneto_mag = sqrt(magneto_data[0] * magneto_data[0] + magneto_data[1] * magneto_data[1] + magneto_data[2] * magneto_data[2]);
				if(magneto_mag > magneto_threshold && warning_flags[3] == 0){
					//printf("Check patient's abnormal orientation! \r\n");
					sprintf(message_print, "%s", "Check patient's abnormal orientation! \r\n");
					HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
					blink_led = 1;
					warning_flags[3] = 1;
				}

				if((humidity_data < humidity_threshold || baro_data > baro_threshold) && warning_flags[4] == 0){
					//printf("Check patient's breath! \r\n");
					sprintf(message_print, "%s", "Check patient's breath! \r\n");
					HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
					blink_led = 1;
					warning_flags[4] = 1;
				}
			}

			/* extension */
			WiFi_Stat = WIFI_GetHostAddress(SERVER_NAME, ipaddr);
			//printf("IP: %d.%d.%d.%d \r\n", ipaddr[0], ipaddr[1], ipaddr[2], ipaddr[3]);

			WiFi_Stat = WIFI_OpenClientConnection(1, WIFI_TCP_PROTOCOL, "conn", ipaddr, DEST_PORT, SOURCE_PORT);

			// poll for operation_mode and update if no changes
			sprintf((char*)req, "GET /rvXK2x3GtPfvJnaNbHA4ln50v2unophS/get/V4 HTTP/1.1\r\nHost: blynk-cloud.com\r\n\r\n");
			WiFi_Stat = WIFI_SendData(1, req, (uint16_t)strlen((char*)req), &Datalen, WIFI_WRITE_TIMEOUT);
			WiFi_Stat = WIFI_ReceiveData(1, resp, MAX_LENGTH, &Datalen, WIFI_READ_TIMEOUT);
			//printf("Datalen: %d \r\n", Datalen);
			resp[Datalen] = '\0'; // to null terminate the response string
			//printf("%s\r\n",(char *)resp);
			if( (Datalen == 0) || !strstr((char *)resp, "200") ){
				while(1);
			}
			else{
				char baro_val[5];
				sprintf(baro_val, "%.*s\r\n", 4, (char *)resp + Datalen - 6);
				baro_threshold = (float)atof(baro_val);
				//printf("%f\r\n", baro_threshold);
			}

			if(HAL_GetTick() - last_led_toggle_tick > 250 && blink_led){
				last_led_toggle_tick = HAL_GetTick();
				HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
			}

			// poll for operation_mode and update if no changes
			sprintf((char*)req, "GET /rvXK2x3GtPfvJnaNbHA4ln50v2unophS/get/V2 HTTP/1.1\r\nHost: blynk-cloud.com\r\n\r\n");
			WiFi_Stat = WIFI_SendData(1, req, (uint16_t)strlen((char*)req), &Datalen, WIFI_WRITE_TIMEOUT);
			WiFi_Stat = WIFI_ReceiveData(1, resp, MAX_LENGTH, &Datalen, WIFI_READ_TIMEOUT);
			//printf("Datalen: %d \r\n", Datalen);
			resp[Datalen] = '\0'; // to null terminate the response string
			//printf("%s\r\n",(char *)resp);
			if( (Datalen == 0) || !strstr((char *)resp, "200") ){
				while(1);
			}
			else{
				if (strstr((char *)resp, "[\"0\"]") && operation_mode == 1){
					mode_changed_flag = 1;
				}
				else if (strstr((char *)resp, "[\"1\"]") && operation_mode == 0){
					mode_changed_flag = 1;
				}
				else{
					// update operation_mode
					sprintf((char*)req, "GET /rvXK2x3GtPfvJnaNbHA4ln50v2unophS/update/V2?value=%d HTTP/1.1\r\nHost: blynk-cloud.com\r\n\r\n", operation_mode);
					WiFi_Stat = WIFI_SendData(1, req, (uint16_t)strlen((char*)req), &Datalen, WIFI_WRITE_TIMEOUT);
				}
			}

			if(HAL_GetTick() - last_led_toggle_tick > 250 && blink_led){
				last_led_toggle_tick = HAL_GetTick();
				HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
			}

			// update temp
			sprintf((char*)req, "GET /rvXK2x3GtPfvJnaNbHA4ln50v2unophS/update/V0?value=%f HTTP/1.1\r\nHost: blynk-cloud.com\r\n\r\n", temp_data);
			WiFi_Stat = WIFI_SendData(1, req, (uint16_t)strlen((char*)req), &Datalen, WIFI_WRITE_TIMEOUT);

			if(HAL_GetTick() - last_led_toggle_tick > 250 && blink_led){
				last_led_toggle_tick = HAL_GetTick();
				HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
			}

			// update accelero
			sprintf((char*)req, "GET /rvXK2x3GtPfvJnaNbHA4ln50v2unophS/update/V1?value=%f HTTP/1.1\r\nHost: blynk-cloud.com\r\n\r\n", accel_mag);
			WiFi_Stat = WIFI_SendData(1, req, (uint16_t)strlen((char*)req), &Datalen, WIFI_WRITE_TIMEOUT);

			if(HAL_GetTick() - last_led_toggle_tick > 250 && blink_led){
				last_led_toggle_tick = HAL_GetTick();
				HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
			}

			if (operation_mode){
				// update baro
				sprintf((char*)req, "GET /rvXK2x3GtPfvJnaNbHA4ln50v2unophS/update/V3?value=%f HTTP/1.1\r\nHost: blynk-cloud.com\r\n\r\n", baro_data);
				WiFi_Stat = WIFI_SendData(1, req, (uint16_t)strlen((char*)req), &Datalen, WIFI_WRITE_TIMEOUT);
			}
			else{
				// update baro
				sprintf((char*)req, "GET /rvXK2x3GtPfvJnaNbHA4ln50v2unophS/update/V3?value=\"NA\" HTTP/1.1\r\nHost: blynk-cloud.com\r\n\r\n");
				WiFi_Stat = WIFI_SendData(1, req, (uint16_t)strlen((char*)req), &Datalen, WIFI_WRITE_TIMEOUT);
			}

			if(HAL_GetTick() - last_led_toggle_tick > 250 && blink_led){
				last_led_toggle_tick = HAL_GetTick();
				HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
			}

			WiFi_Stat = WIFI_CloseClientConnection(1);
		}

		if(current_tick - last_10s_tick > 10000){
			last_10s_tick = current_tick;

			// raise healthy warnings
			if(warning_flags[0] == 1){
				//printf("Falling is detected \r\n");
				sprintf(message_print, "%s", "Falling is detected \r\n");
				HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
			}

			if(warning_flags[1] == 1){
				//printf("Fever is detected \r\n");
				sprintf(message_print, "%s", "Fever is detected \r\n");
				HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
			}

			if(operation_mode == 1){
				// raise ICU warnings
				if(warning_flags[2] == 1){
					//printf("Patient in pain! \r\n");
					sprintf(message_print, "%s", "Patient in pain! \r\n");
					HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
				}

				if(warning_flags[3] == 1){
					//printf("Check patient's abnormal orientation! \r\n");
					sprintf(message_print, "%s", "Check patient's abnormal orientation! \r\n");
					HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
				}

				if(warning_flags[4] == 1){
					//printf("Check patient's breath! \r\n");
					sprintf(message_print, "%s", "Check patient's breath! \r\n");
					HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
				}

				//printf("%03d_TEMP_%06.2f_", msg_count, temp_data);
				//printf("ACC_%4.2f_%4.2f_%4.2f \r\n", accel_data[0], accel_data[1], accel_data[2]);
				sprintf(message_print, "%03d_TEMP_%06.2f_ACC_%4.2f_%4.2f_%4.2f \r\n", msg_count, temp_data, accel_data[0], accel_data[1], accel_data[2]);
				HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);

				//printf("%03d GYRO %5.1f ", msg_count, gyro_data);
				//printf("MAGNETO %4.2f %4.2f %4.2f \r\n", magneto_data[0], magneto_data[1], magneto_data[2]);
				sprintf(message_print, "%03d GYRO %5.1f MAGNETO %4.2f %4.2f %4.2f \r\n", msg_count, gyro_data, magneto_data[0], magneto_data[1], magneto_data[2]);
				HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);

				//printf("%03d HUMIDITY %4.2f ", msg_count++, humidity_data);
				//printf("BARO %4.2f \r\n", baro_data);
				sprintf(message_print, "%03d HUMIDITY %4.2f BARO %4.2f \r\n", msg_count++, humidity_data, baro_data);
				HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
			}
		}


		// check for mode change from button press
		if(mode_changed_flag == 1){
			int i;

			// reset LED
			blink_led = 0;
			HAL_GPIO_WritePin(GPIOB, LED2_Pin, GPIO_PIN_RESET);

			// reset warning flags
			for (i = 0; i < 5; i++){
				warning_flags[i] = 0;
			}

			if(operation_mode == 0){
				// change mode from healthy to intensive care
				operation_mode = 1;
				//printf("Entering Intensive Care Mode.\r\n");
				sprintf(message_print, "%s", "Entering Intensive Care Mode.\r\n");
				HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
			}

			else if(operation_mode == 1){
				// change mode from intensive care to healthy
				operation_mode = 0;

				//printf("Entering Healthy Mode.\r\n");
				sprintf(message_print, "%s", "Entering Healthy Mode.\r\n");
				HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
			}

			/* extension */
			WiFi_Stat = WIFI_GetHostAddress(SERVER_NAME, ipaddr);
			//printf("IP: %d.%d.%d.%d \r\n", ipaddr[0], ipaddr[1], ipaddr[2], ipaddr[3]);

			WiFi_Stat = WIFI_OpenClientConnection(1, WIFI_TCP_PROTOCOL, "conn", ipaddr, DEST_PORT, SOURCE_PORT);

			// update operation_mode
			sprintf((char*)req, "GET /rvXK2x3GtPfvJnaNbHA4ln50v2unophS/update/V2?value=%d HTTP/1.1\r\nHost: blynk-cloud.com\r\n\r\n", operation_mode);
			WiFi_Stat = WIFI_SendData(1, req, (uint16_t)strlen((char*)req), &Datalen, WIFI_WRITE_TIMEOUT);
			WiFi_Stat = WIFI_CloseClientConnection(1);

			mode_changed_flag = 0;
		}

		if(accelero_flag == 1){
			//printf("Falling is detected by interrupt\r\n");
			sprintf(message_print, "%s", "Falling is detected (by interrupt)\r\n");
			HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
			blink_led = 1;
			warning_flags[0] = 1;
			accelero_flag = 0;
		}
	}
}

static void MX_GPIO_Init(void) // YQ: added GPIO initialisation function
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, LED2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin LED2_Pin */
	GPIO_InitStruct.Pin = LED2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(BUTTON_EXTI13_GPIO_Port, BUTTON_EXTI13_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin BUTTON_Pin */
	GPIO_InitStruct.Pin = BUTTON_EXTI13_Pin;
//	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(BUTTON_EXTI13_GPIO_Port, &GPIO_InitStruct);

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LSM6DSL_INT1_EXTI11_GPIO_Port, LSM6DSL_INT1_EXTI11_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin ACCELERO_Pin */
	GPIO_InitStruct.Pin = LSM6DSL_INT1_EXTI11_Pin;
//	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LSM6DSL_INT1_EXTI11_GPIO_Port, &GPIO_InitStruct);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == GPIO_PIN_1){
		SPI_WIFI_ISR();
	}

	if (GPIO_Pin == GPIO_PIN_11){
		accelero_flag = 1;
	}

	if (GPIO_Pin == GPIO_PIN_13){
		if(operation_mode == 0){
			if(HAL_GetTick() - last_button_press_tick < 1000){
				mode_changed_flag = 1;
			}
		}
		if(operation_mode == 1){
			mode_changed_flag = 1;
		}
		last_button_press_tick = HAL_GetTick();
	}
}

void Accelero_Int_Init(void){
	uint8_t tmp;

	tmp = SENSOR_IO_Read(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_TAP_CFG1);
	tmp |= (1 << 7);
	SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_TAP_CFG1, tmp);

	tmp = SENSOR_IO_Read(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_FREE_FALL);
	tmp |= 0x07;
	SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_FREE_FALL, tmp);

	tmp = SENSOR_IO_Read(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_MD1_CFG);
	tmp |= (1 << 4);
	SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_MD1_CFG, tmp);
}

static void UART1_Init(void)
{
    /* Pin configuration for UART. BSP_COM_Init() can do this automatically */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Configuring UART1 */
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
      while(1);
    }

}

void SPI3_IRQHandler(void)
{
    HAL_SPI_IRQHandler(&hspi3);
}
