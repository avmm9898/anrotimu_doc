#include <string.h>
#include <stdio.h>

#include "packet.h"
#include "imu_data_decode.h"

static packet_t RxPkt; /* used for data receive */

uint8_t bitmap;

__align(4)  receive_imusol_packet_t receive_imusol;
__align(4)  receive_gwsol_packet_t receive_gwsol;

static int stream2int16(int *dest,uint8_t *src)
{
	dest[0] = (int16_t)(src[0] | src[1] << 8);
	dest[1] = (int16_t)(src[2] | src[3] << 8);
	dest[2] = (int16_t)(src[4] | src[5] << 8);	
	return 0;
}   


/*  callback function of  when recv a data frame successfully */
static void on_data_received(packet_t *pkt)
{
	int temp[3] = {0};
    int i = 0;
    int offset = 0;
    uint8_t *p = pkt->buf;

	if(pkt->type != 0xA5)
    {
        return;
    }
	
	bitmap = 0;
	while(offset < pkt->payload_len)
	{
		switch(p[offset])
		{
            case kItemID:
                bitmap |= BIT_VALID_ID;
                receive_imusol.id = p[1];
                offset += 2;
                break;
				
            case kItemAccRaw:
                bitmap |= BIT_VALID_ACC;
                stream2int16(temp, p + offset + 1);
                receive_imusol.acc[0] = (float)temp[0] / 1000;
                receive_imusol.acc[1] = (float)temp[1] / 1000;
                receive_imusol.acc[2] = (float)temp[2] / 1000;
                offset += 7;
                break;
				
            case kItemGyrRaw:
			case kItemGyrRaw_yunjing:
                bitmap |= BIT_VALID_GYR;
                stream2int16(temp, p + offset + 1);
                receive_imusol.gyr[0] = (float)temp[0] / 10;
                receive_imusol.gyr[1] = (float)temp[1] / 10;
                receive_imusol.gyr[2] = (float)temp[2] / 10;
                offset += 7;
                break;
				
            case kItemMagRaw:
                bitmap |= BIT_VALID_MAG;
                stream2int16(temp, p + offset + 1);
                receive_imusol.mag[0] = (float)temp[0] / 10;
                receive_imusol.mag[1] = (float)temp[1] / 10;
                receive_imusol.mag[2] = (float)temp[2] / 10;
                offset += 7;
                break;
				
            case kItemRotationEul:
                bitmap |= BIT_VALID_EUL;
                stream2int16(temp, p + offset + 1);
                receive_imusol.eul[1] = (float)temp[0] / 100;
                receive_imusol.eul[0] = (float)temp[1] / 100;
                receive_imusol.eul[2] = (float)temp[2] / 10;
                offset += 7;
                break;
				
            case kItemRotationQuat:
                bitmap |= BIT_VALID_QUAT;
                memcpy((void *)receive_imusol.quat, p + offset + 1, sizeof( receive_imusol.quat));
                offset += 17;
                break;
				
            case kItemPressure:
                offset += 5;
                break;

            case KItemIMUSOL:
                bitmap = BIT_VALID_ALL;
                receive_imusol.id =p[offset + 1];
                memcpy((void *) receive_imusol.acc, p + 12 , sizeof(float) * 16);
                offset += 76;
                break;
				
            case KItemGWSOL:
                receive_gwsol.tag = p[offset];
                receive_gwsol.gw_id = p[offset + 1]; 
                receive_gwsol.n = p[offset + 2];
                offset += 8;
                for (i = 0; i < receive_gwsol.n; i++)
                {
                    bitmap = BIT_VALID_ALL;
                    receive_gwsol.receive_imusol[i].tag = p[offset];
                    receive_gwsol.receive_imusol[i].id = p[offset + 1];
                    memcpy(&receive_gwsol.receive_imusol[i].acc, p + offset + 12 , sizeof(float) * 16);
                    offset += 76;
                }
                break;
				
            default:
				offset++;
		}
    }
}


int imu_data_decode_init(void)
{
    packet_decode_init(&RxPkt, on_data_received);
    return 0;
}



