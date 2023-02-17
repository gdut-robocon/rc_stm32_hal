#include "HareWare.h"
#include "main.h"
void MotorInit(void)
{
	//Çý¶¯Æ÷³õÊ¼»¯
	ElmoInit(&hcan1);
	ElmoInit(&hcan2);

}
