#include "delay.h"
#include "usart.h"
#include "imu_data_decode.h"
#include "packet.h"
 
/************************************************
 ALIENTEK战舰STM32开发板实验
 串口实验 
 技术支持：www.openedv.com
 淘宝店铺：http://eboard.taobao.com 
 关注微信公众平台微信号："正点原子"，免费获取STM32资料。
 广州市星翼电子科技有限公司  
 作者：正点原子 @ALIENTEK
************************************************/

/************************************************
 北京超核电子
 惯性导航模块： HI226 HI229
 串口接收数据例程
 本例程只供学习使用，观察数据输出，不做其他用途
 串口2接收来自HI226或者是HI229的数据
 串口1将串口2成功接收到的数据打印到终端上
 这里的终端一般指的是PC机上串口调试助手
 官网：http://www.hipnuc.com
************************************************/

void dump_data_packet(receive_imusol_packet_t *data);     //打印数据
void SysTick_Handler(void);                               //SysTick中断处理函数
void SysTick_Init(void);                                  //SysTick初始化函数

static uint32_t frame_rate;                               //获取帧频率
static uint8_t usart1_output_flag;                        //向终端输出标志

int main(void)
{		
    uint32_t i = 0;

    delay_init();	    	                               //延时函数初始化	  
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);        //设置NVIC中断分组2:2位抢占优先级，2位响应优先级
    uart_init(115200);	                                   //串口初始化为115200
  
    imu_data_decode_init();                                //映射解析函数
    SysTick_Init();                                        //开启SysTick，定时20ms

    while(1)
    {
        if(usart1_output_flag)
        {
            usart1_output_flag = 0;
            if(receive_gwsol.tag != KItemGWSOL)
			{
				/* printf imu data packet */
				dump_data_packet(&receive_imusol);
				putchar(10);
			}
			else
			{
				/* printf gw data packet */
				printf("        GW ID:  %-8d\n",receive_gwsol.gw_id);
				for(i = 0; i < receive_gwsol.n; i++)
				{ 
					dump_data_packet(&receive_gwsol.receive_imusol[i]);
					puts("");
				}
			}
		}
	}	 
}

/* 200ms interrupt */
void SysTick_Handler(void)
{
	static uint32_t div;    
	if(div == 5)
	{
		div = 0;
		frame_rate = frame_count;
		frame_count = 0;
		usart1_output_flag = 1;
	}
    
	div++;
}

/* serial usart2 interrupt functional */
void USART2_IRQHandler(void)                	            //串口2中断服务程序
{
	uint8_t ch;
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)   //接收中断 
		ch = USART_ReceiveData(USART2);	                    //读取接收到的数据

	packet_decode(ch);                                      //解析数据
} 

/* SysTick initialization */
void SysTick_Init(void)
{
    SysTick->LOAD = (float)SystemCoreClock / 40;             
    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk; 
}

/* printf hi229 or hi226 data packet*/
void dump_data_packet(receive_imusol_packet_t *data)
{
	if(bitmap & BIT_VALID_ID)
		printf("    Device ID:  %-8d\r\n",  data->id);
	printf("   Frame Rate: %4dHz\r\n", frame_rate);
	if(bitmap & BIT_VALID_ACC)
		printf("       Acc(G):	%8.3f %8.3f %8.3f\r\n",  data->acc[0],  data->acc[1],  data->acc[2]);
	if(bitmap & BIT_VALID_GYR)
		printf("   gyr(deg/s):	%8.2f %8.2f %8.2f\r\n",  data->gyr[0],  data->gyr[1],  data->gyr[2]);
	if(bitmap & BIT_VALID_MAG)
		printf("      mag(uT):	%8.2f %8.2f %8.2f\r\n",  data->mag[0],  data->mag[1],  data->mag[2]);
	if(bitmap & BIT_VALID_EUL)
		printf("   eul(R P Y):  %8.2f %8.2f %8.2f\r\n",  data->eul[0],  data->eul[1],  data->eul[2]);
	if(bitmap & BIT_VALID_QUAT)
		printf("quat(W X Y Z):  %8.3f %8.3f %8.3f %8.3f\r\n",  data->quat[0],  data->quat[1],  data->quat[2],  data->quat[3]);
}
