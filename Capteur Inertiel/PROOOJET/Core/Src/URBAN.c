/*
 * URBAN.c
 *
 *  Created on: Apr 19, 2022
 *      Author: alixh
 */
#include "URBAN.h"
#include "main.h"
#include "math.h"
#include "stdio.h"


extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim2;
extern uint8_t INTBTN;
extern uint8_t INTTIM12;


uint8_t buf[10];


void URBAN(){

	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	/* USER CODE BEGIN 2 */

	for (int i = 0; i < 10; i++)
		buf[i] = 0;

	HAL_StatusTypeDef r = HAL_I2C_IsDeviceReady(&hi2c1, ADDRESS_IMU, 5, 2);

	// ===== Verif Id accel/gyro
	buf[0] = 117; // register address (def val 0x71)
	r = HAL_I2C_Master_Transmit(&hi2c1, ADDRESS_IMU, buf, 1, 200);

	//on se positionne à l'adress imu et on lit 1 bit
	r = HAL_I2C_Master_Receive(&hi2c1, ADDRESS_IMU, buf, 1, 200);

	buf[0]=IMU_IN_PIN_CFG_REG;
	buf[1]=BYPASS_EN;
	r=HAL_I2C_Master_Transmit(&hi2c1, ADDRESS_IMU, buf, 2, 200);

	// ===== Verif Id Mag
	buf[0] = 0x00; // register address (def val 0x71)
	r = HAL_I2C_Master_Transmit(&hi2c1, ADDRESS_MAG, buf, 1, 200);
	//on se positionne à l'adress imu et on lit 1 bit
	r = HAL_I2C_Master_Receive(&hi2c1, ADDRESS_MAG, buf, 1, 200);
	// Test: si buf[0] != de valeur doc (0x48) =>>> Kernel Panic

	buf[0] = IMU_PWR_MGMT_1_REG;
	buf[1] = 0;
	r = HAL_I2C_Master_Transmit(&hi2c1, ADDRESS_IMU, buf, 2, 200);

	//on ecrit dans le registre 107 la valeur 0 qui permet d'activer l' IMU
	buf[0] = 107;
	r = HAL_I2C_Master_Receive(&hi2c1, ADDRESS_IMU, buf, 1, 200);


	// on écrit dans le registre 0x37 la valeur 1 pour activer le "bypass mode"
	buf[0] = 0;
	r = HAL_I2C_Mem_Write(&hi2c1, ADDRESS_IMU, IMU_PWR_MGMT_1_REG,I2C_MEMADD_SIZE_8BIT, (uint8_t*) buf, 1, 10);



	/* USER CODE END 2 */

	/* Infinite loop */

	//magnétomètre selon x
	uint8_t AK8963_XOUT_L, AK8963_XOUT_H;
	int16_t AK8963_XOUT;


	//magnétomètre selon y
	uint8_t AK8963_YOUT_L, AK8963_YOUT_H;
	int16_t AK8963_YOUT;

	double POSITION_DEGRE;
	double POSITION_RADIAN;
	double REF_DEGRE;


	while (1) {

		//Valeur pour la boussole
		uint8_t rawData[14];
		r = HAL_I2C_Mem_Read(&hi2c1, ADDRESS_MAG, FIRST_REG_MAG, I2C_MEMADD_SIZE_8BIT, rawData, 14, 10);

		AK8963_XOUT_L=rawData[0];
		AK8963_XOUT_H=rawData[1];
		AK8963_XOUT=(AK8963_XOUT_H << 8) + AK8963_XOUT_L;

		AK8963_YOUT_L=rawData[2];
		AK8963_YOUT_H=rawData[3];
		AK8963_YOUT=(AK8963_YOUT_H << 8) + AK8963_YOUT_L;

		//Calcul de la position en degré

		POSITION_RADIAN = atan2(AK8963_YOUT,AK8963_XOUT);
		POSITION_DEGRE = (180*POSITION_RADIAN)*M_PI;


		HAL_I2C_Mem_Write(&hi2c1, ADDRESS_MAG, AK8963_CNTL,I2C_MEMADD_SIZE_8BIT, buf, 1, 10); // Power down magnetometer
		HAL_Delay(10);
		buf[0] = 0x0F;
		HAL_I2C_Mem_Write(&hi2c1, ADDRESS_MAG, AK8963_CNTL,I2C_MEMADD_SIZE_8BIT, buf, 1, 10); // Enter Fuse ROM access mode
		HAL_Delay(10);

		if (INTBTN == 1)//Si on appuie sur le boutton
			{
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
			REF_DEGRE=POSITION_DEGRE;//On prend une valeur de référence
			INTBTN=0;
			}//INTBTN==1


		if(INTTIM12==1){
			// Script exécuté toutes les secondes
			if(REF_DEGRE<POSITION_DEGRE){
				//HAL_TIM_Base_Start(&htim2);
				//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 1);
				/*	if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK)//Starts the PWM signal generation
				{
			*/	}
				//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 0);
			}
			if(REF_DEGRE>POSITION_DEGRE){
				// Activation autre vibreur
				}

			INTTIM12 = 0;
			}


/*		Code activation LED sur PB3

 * HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 1);//LED de PB3 sur 1
		HAL_Delay(500);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 0);
		HAL_Delay(500);
*/


		buf[0] = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, ADDRESS_MAG, AK8963_CNTL,I2C_MEMADD_SIZE_8BIT, buf, 1, 10); // Power down magnetometer
		HAL_Delay(10);


		// Configure the magnetometer for continuous read and highest resolution
		// set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
		// and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
		int Mscale = 1;//resolution de mesure : 16 bits
		int Mmode = 2;//0010 "continuous maesurement mode 1
		buf[0] = Mscale << 4 | Mmode;
		HAL_I2C_Mem_Write(&hi2c1, ADDRESS_MAG, AK8963_CNTL,I2C_MEMADD_SIZE_8BIT, buf, 1, 10); // Power up magnetometer
		HAL_Delay(10);

	//	sprintf(str, "%f\r\n", POSITION_RADIAN);
	//	r = HAL_UART_Transmit(&huart2, str, strlen(str), 100);
	//	HAL_Delay(200);

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}



