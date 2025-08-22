#include "stm32f4xx_hal.h"

//#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))



float PID_velocity(float error);
float Limit(float x,float y);

