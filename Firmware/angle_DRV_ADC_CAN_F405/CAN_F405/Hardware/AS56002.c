#include "as56002.h"
#include "i2c.h"//使用硬件I2C

extern  UART_HandleTypeDef huart1;//这个开头值得借鉴，这种写法可以避免错误


void test_as5600()//检测器件地址
{
uint8_t a1=0x00;
uint8_t a2=0x55;	
	for(int i=0;i<255;i++)
	{
	if(HAL_I2C_Master_Transmit(&hi2c1,a1,&a2,1,100)==HAL_OK)
     
	printf("AT 0x%x have a mod\r\n",a1);
     
	a1++;
	}
}

uint8_t readOneByte(uint8_t in_adr)//读一个字节函数，从5600的指定寄存器提取一个8位值
{
uint8_t a3=0;
HAL_I2C_Master_Transmit(&hi2c1,0x6c,&in_adr,1,100);
HAL_I2C_Master_Receive(&hi2c1,0x6c,&a3,1,100);
//printf("接收成功\r\n");
return a3;
}

uint16_t readtwoByte(uint8_t h_adr , uint8_t l_adr)
{
uint8_t high=0;
uint8_t low=0;	
uint16_t retVal = 0;
	
high = readOneByte(h_adr);
low = readOneByte(l_adr);	
	
retVal = high << 8;
retVal = retVal | low;

return retVal;//输出一个16位


}

int16_t getRawAngle(void)
{
  return readtwoByte(_raw_ang_hi, _raw_ang_lo);
}
