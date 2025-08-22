#include "PID.h"
#include "FOC.h"

//转速PID
float KP_vel=0.04;                         //0.06     20r:0.12           NB:0.04
float KI_vel=0;                        //0.001     20r:0.02
float KD_vel=0.0001;                      // 0.0001      0.0000458       NB:0.0001
float voltage_limit=10;                   //转速环输出(Uq)限幅15
float Ts=0.001;                          //1ms的控制周期
 float integral_vel_prev=0;

 float y_vel_prev=0;
float vel_LPF=0;

//角度PID
float KP_ang=0.005;
float KI_ang=0;
float KD_ang=0;
float integral_ang_prev=0;
float speed_limit=10;                 //10rad/s限幅

//电流PID
float KP_Id=0.01;
float KI_Id=0.0;
float KD_Id=0;
float integral_Id_prev=0;
float Ud_limit=1;

float KP_Iq=0.7;                  //0.55  0.8
float KI_Iq=0.008;                //0.04   0.045
float KD_Iq=0;
float integral_Iq_prev=0;
float Uq_limit=8;                    //上限13.8

extern float SOA_value;
extern float SOB_value;
extern float SOC_value;

extern float vel_sp;

float Limit(float x,float y)
{
	if(x>y) x=y;
	else if(x<-y) x=-y;
	return x;
}


float LPF_velocity(float x)               //反馈转速低通滤波
{
	float y = 0.9*y_vel_prev + 0.1*x;
	
	y_vel_prev=y;
	
	return y;
}

//************************************************转速PID************************************************//
float PID_velocity(float error)          //转速PID
{
	float output, output_vel_prev;
	float proportional, integral, derivative;
	float error_vel_prev;
	
	proportional = KP_vel * error;
	integral = integral_vel_prev + KI_vel * Ts * error;
	derivative = KD_vel*(error - error_vel_prev)/Ts;
	//integral = Limit(integral,2);
	
	output = proportional + integral + derivative;
	output = Limit(output,voltage_limit);
	
	integral_vel_prev = integral;      //存储前次PID
	output_vel_prev = output;
	error_vel_prev = error;
	
	return output;
}

//************************************************角度PID************************************************//
float PID_angle(float error)          //转速PID
{
	float output, output_ang_prev;
	float proportional, integral, derivative;
	float error_ang_prev;
	
	proportional = KP_ang * error;
	integral = integral_ang_prev + KI_ang * Ts * error;
	derivative = KD_ang*(error - error_ang_prev)/Ts;
	
	output = proportional + integral + derivative;
	output = Limit(output,speed_limit);
	
	integral_ang_prev = integral;      //存储前次PID
	output_ang_prev = output;
	error_ang_prev = error;
	
	return output;
}
//************************************************电流PID************************************************//
float PID_Id(float error)          //转速PID
{
	float output, output_Id_prev;
	float proportional, integral, derivative;
	float error_Id_prev;
	
	proportional = KP_Id * error;
	integral = integral_Id_prev + KI_Id * Ts * error;
	derivative = KD_vel*(error - error_Id_prev)/Ts;
	//integral = Limit(integral,2);
	output = proportional + integral + derivative;
	output = Limit(output,Ud_limit);
	
	integral_Id_prev = integral;      //存储前次PID
	output_Id_prev = output;
	error_Id_prev = error;
	
	return output;
}

float PID_Iq(float errorq)          //转速PID
{
	float output, output_Iq_prev;
	float proportional, integral, derivative;
	float error_Iq_prev;
	
	
	proportional = KP_Iq * errorq;
	integral = integral_Iq_prev + KI_Iq * Ts * errorq;
	derivative = KD_Iq*(errorq - error_Iq_prev)/Ts;
	output = proportional + integral + derivative;
	output = Limit(output,Uq_limit);
	
	integral_Iq_prev = integral;      //存储前次PID
	output_Iq_prev = output;
	error_Iq_prev = errorq;
	
	return output;
}
