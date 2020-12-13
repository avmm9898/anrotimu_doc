#ifndef __IMU_DATA_DECODE_H__
#define __IMU_DATA_DECODE_H__

#include <stdint.h>
#include <stdbool.h>
#define MAX_LENGTH 16

extern uint8_t bitmap;
#define BIT_VALID_ID     (0x01)
#define BIT_VALID_ACC    (0x02)
#define BIT_VALID_GYR    (0x04)
#define BIT_VALID_MAG    (0x08)
#define BIT_VALID_EUL    (0x10)
#define BIT_VALID_QUAT   (0x20)
#define BIT_VALID_TIMES  (0x40)
#define BIT_VALID_ALL  (BIT_VALID_QUAT | BIT_VALID_EUL | BIT_VALID_MAG | BIT_VALID_GYR | BIT_VALID_ACC | BIT_VALID_ID | BIT_VALID_TIMES)

typedef struct receive_imusol_packet_t {
	uint8_t tag;
	uint8_t id;
	uint32_t times;
	float acc[3];
	float gyr[3];
	float mag[3]; 
	float eul[3];
	float quat[4];

}__attribute__((packed())) receive_imusol_packet_t;

typedef struct receive_gwsol_packet_t {
	uint8_t tag;
	uint8_t gw_id;
	uint8_t n;
	receive_imusol_packet_t receive_imusol[MAX_LENGTH];
}__attribute__((packed())) receive_gwsol_packet_t;
	 
extern receive_imusol_packet_t receive_imusol;
extern receive_gwsol_packet_t receive_gwsol;

typedef enum 
{
    kItemID =                   0x90,   /* user programed ID   */
    kItemAccRaw =               0xA0,   /* raw acc             */
    kItemGyrRaw =               0xB0,   /* raw gyro            */  
	kItemGyrRaw_yunjing =       0xB1,   /* raw gyro yunjing    */
    kItemMagRaw =               0xC0,   /* raw mag             */
    kItemRotationEul =          0xD0,   /* eular angle         */
    kItemRotationQuat =         0xD1,   /* att q               */
    kItemPressure =             0xF0,   /* pressure            */
    kItemEnd =                  0x00,   
	KItemIMUSOL =               0x91,   /* IMUSOL  */
	KItemGWSOL =                0x62,   /* RFSOL  */
}ItemID_t;

int imu_data_decode_init(void);

#endif 

 
