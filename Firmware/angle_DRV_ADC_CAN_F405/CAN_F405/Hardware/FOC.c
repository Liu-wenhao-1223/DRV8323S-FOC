#include "time.h"
#include "math.h"
#include "FOC.h"

float voltage_power_supply;

float _normalizeAngle(float angle)
{
  float a = fmod(angle, _2PI);                     //fmod()�Ը�����ȡģ
  return a >= 0 ? a : (a + _2PI);
}
float CurrentCircule_Id(float Ia, float Ib, float Ic, float angle_el)
{
	float I_alpha, I_beta;
	float Id;
	
	I_alpha = Ia - 0.5*Ib - 0.5*Ic;                        //Clark�任
	I_beta = _SQRT3/2*(Ib - Ic);
	
	angle_el = _normalizeAngle(angle_el);
	
	Id = I_alpha*_cos(angle_el) + I_beta*_sin(angle_el);     //park�任
	//Iq = -I_alpha*sin(angle_el) + I_beta*cos(angle_el);
  
	return Id;
}

float CurrentCircule_Iq(float Ia, float Ib, float Ic, float angle_el)
{
	float I_alpha, I_beta;
	float Iq;
	
	I_alpha = Ia - 0.5*Ib - 0.5*Ic;                        //Clark�任
	I_beta = _SQRT3/2*(Ib - Ic);
	
	angle_el = _normalizeAngle(angle_el);
	
	//Id = I_alpha*cos(angle_el) + I_beta*sin(angle_el);     //park�任
	Iq = -I_alpha*_sin(angle_el) + I_beta*_cos(angle_el);
  
	return Iq;
}

//FOC���ĺ���������Ud��Uq�͵�Ƕȣ����PWM
void setPhaseVoltage(float Uq, float Ud, float angle_el)
{
	float Uref;   //
	float U_alpha, U_beta;
	float T0,T1,T2;
	float Ta,Tb,Tc,Td;
	float hDeltaDuty;
	int sector;
	
	if(Uq>0)
	  angle_el = _normalizeAngle(angle_el+_PI_2);           //��׼����Ƕ�ֵ��0,2pi������90�Ⱥ��ǲο���ѹʸ����λ��
	else
		angle_el = _normalizeAngle(angle_el-_PI_2);
	
	U_alpha=Ud*_cos(angle_el)-Uq*_sin(angle_el);            //��park�任
	U_beta=Ud*_sin(angle_el)+Uq*_cos(angle_el);
	
	Uref=_sqrt(U_alpha*U_alpha + U_beta*U_beta) / voltage_power_supply;    //����ο���ѹʸ���ķ�ֵ
  if(Uref> 0.577)Uref= 0.577;                     			//�����ε�����Բ(SVPWM���ʧ����ת��ѹʸ����ֵ)����3/3
	if(Uref<-0.577)Uref=-0.577; 
	
	//�жϲο���ѹʸ������������         
	sector = (angle_el / _PI_3) + 1; 
	
	T1 = _SQRT3*_sin(sector*_PI_3 - angle_el) * Uref;           //�����������ڵ�ѹʸ������ʱ��
	T2 = _SQRT3*_sin(angle_el - (sector-1.0)*_PI_3) * Uref;
	T0 = 1 - T1 - T2;                                          //��ʸ������ʱ��

  switch(sector)
	{
		case 1:
			Ta = T1 + T2 + T0/2;
			Tb = T2 + T0/2;
			Tc = T0/2;
		  
			break;
		case 2:
			Ta = T1 +  T0/2;
			Tb = T1 + T2 + T0/2;
			Tc = T0/2;
			break;
		case 3:
			Ta = T0/2;
			Tb = T1 + T2 + T0/2;
			Tc = T2 + T0/2;
			break;
		case 4:
			Ta = T0/2;
			Tb = T1+ T0/2;
			Tc = T1 + T2 + T0/2;
			break;
		case 5:
			Ta = T2 + T0/2;
			Tb = T0/2;
			Tc = T1 + T2 + T0/2;
			break;
		case 6:
			Ta = T1 + T2 + T0/2;
			Tb = T0/2;
			Tc = T1 + T0/2;
			break;
		default:  // possible error state
			Ta = 0;
			Tb = 0;
			Tc = 0;
	}
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,Ta*PWM_Period);      //���PWM,����ռ�ձ�
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,Tb*PWM_Period);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,Tc*PWM_Period);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,95);
}
const int sine_array[200] = {0,79,158,237,316,395,473,552,631,710,789,867,946,1024,1103,1181,1260,1338,1416,1494,1572,1650,1728,1806,1883,1961,2038,2115,2192,2269,2346,2423,2499,2575,2652,2728,2804,2879,2955,3030,3105,3180,3255,3329,3404,3478,3552,3625,3699,3772,3845,3918,3990,4063,4135,4206,4278,4349,4420,4491,4561,4631,4701,4770,4840,4909,4977,5046,5113,5181,5249,5316,5382,5449,5515,5580,5646,5711,5775,5839,5903,5967,6030,6093,6155,6217,6279,6340,6401,6461,6521,6581,6640,6699,6758,6815,6873,6930,6987,7043,7099,7154,7209,7264,7318,7371,7424,7477,7529,7581,7632,7683,7733,7783,7832,7881,7930,7977,8025,8072,8118,8164,8209,8254,8298,8342,8385,8428,8470,8512,8553,8594,8634,8673,8712,8751,8789,8826,8863,8899,8935,8970,9005,9039,9072,9105,9138,9169,9201,9231,9261,9291,9320,9348,9376,9403,9429,9455,9481,9506,9530,9554,9577,9599,9621,9642,9663,9683,9702,9721,9739,9757,9774,9790,9806,9821,9836,9850,9863,9876,9888,9899,9910,9920,9930,9939,9947,9955,9962,9969,9975,9980,9985,9989,9992,9995,9997,9999,10000,10000};
float _sin(float a){
  if(a < _PI_2){
    //return sine_array[(int)(199.0*( a / (_PI/2.0)))];
    //return sine_array[(int)(126.6873* a)];           // float array optimized
    return 0.0001*sine_array[_round(126.6873* a)];      // int array optimized
  }else if(a < _PI){
    // return sine_array[(int)(199.0*(1.0 - (a-_PI/2.0) / (_PI/2.0)))];
    //return sine_array[398 - (int)(126.6873*a)];          // float array optimized
    return 0.0001*sine_array[398 - _round(126.6873*a)];     // int array optimized
  }else if(a < _3PI_2){
    // return -sine_array[(int)(199.0*((a - _PI) / (_PI/2.0)))];
    //return -sine_array[-398 + (int)(126.6873*a)];           // float array optimized
    return -0.0001*sine_array[-398 + _round(126.6873*a)];      // int array optimized
  } else {
    // return -sine_array[(int)(199.0*(1.0 - (a - 3*_PI/2) / (_PI/2.0)))];
    //return -sine_array[796 - (int)(126.6873*a)];           // float array optimized
    return -0.0001*sine_array[796 - _round(126.6873*a)];      // int array optimized
  }
}
/***************************************************************************/
// function approximating cosine calculation by using fixed size array
// ~55us (float array)
// ~56us (int array)
// precision +-0.005
// it has to receive an angle in between 0 and 2PI
float _cos(float a){
  float a_sin = a + _PI_2;
  a_sin = a_sin > _2PI ? a_sin - _2PI : a_sin;
  return _sin(a_sin);
}
/***************************************************************************/

	