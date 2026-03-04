#include <CoVAPSy_LiDAR.h>
#include "tim.h"
#include "stdint.h"
#include "usart.h"

// Initialisation du LiDAR SLAMTECH A2M12
void LiDAR_init(void) {
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); //Lidar
	HAL_Delay(100);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 300); 			//démarrage du lidar par PWM
	HAL_Delay(2000);
	uint8_t packet_CMD_START_SCAN[2] = { 0xA5, 0x20 };
	HAL_Delay(5);
	HAL_UART_Transmit(&huart1, packet_CMD_START_SCAN, 2, 100);	// Start Lidar
	HAL_UART_Receive_IT(&huart1, &Data_RX_LIDAR, 1);

}




