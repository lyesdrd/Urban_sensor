/*
 * URBAN.h
 *
 *  Created on: Apr 19, 2022
 *      Author: alixh
 */

#ifndef INC_URBAN_H_
#define INC_URBAN_H_

//static const uint8_t ADRESS_IMU = (0x77 << 1);
#define ADDRESS_IMU  (0x68 << 1) //adresse du MPU 9250
#define ADDRESS_MAG (0x0C<<1)//adresse boussole

#define IMU_ID_REG  0x75
#define IMU_ID_VAL  0x71 // = 117 en decimal
//WHO_I_AM en position 117 qui doit contenir la valeur 0x68
#define IMU_IN_PIN_CFG_REG 0x37

#define IMU_PWR_MGMT_1_REG  0x6B
#define IMU_PWR_MGMT_2_ 0x6

#define AK8963_CNTL		 0x0A  // P
#define FIRST_REG_MAG	0x03

#define BYPASS_EN 0b00000010

#define ADDRESS_ASAX		0x10  // Fuse ROM x-axis sensitivity adjustment value
#define ADDRESS_ASAY		0x11  // Fuse ROM y-axis sensitivity adjustment value
#define ADDRESS_ASAZ		0x12  // Fuse ROM z-axis sensitivity adjustment value

#define FIRST_REG_IMU 59

//static const ou #define trouver lequel est le mieux GOOGLE EST MON AMI
//Adresse I2C sur 16 bits, bit de poids forts et de poids faible, on laisse de la place pour ce bit la


void URBAN();


#endif /* INC_URBAN_H_ */
