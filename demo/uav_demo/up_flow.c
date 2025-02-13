#include "include.h"
#include "usart.h"
#include "pid.h"
#include "up_flow.h"
#include "mpu6050.h"


#define FLOW_INTERFACE_V1_30

#define POS_CALC_OUT_MAX 	2.5f    //单位
#define POS_CALC_OUT_MIN 	-2.5f


#define POS_CONTROL_LIMIT_MAX 	100 /*单位是厘米*/
#define POS_CONTROL_LIMIT_MIN 	-100

static unsigned char flow_buffer[50];
static unsigned char uart_flag=0;
struct flow_integral_frame  flow_data_frame;

struct flow_float flow_dat;
float distance;
float exp_rol_flow,exp_pit_flow;
float speed_x=0.0,speed_y=0.0;
float sum_flow_x=0.0,sum_flow_y=0.0;
float pos_pid_out_x = 0.0,pos_pid_out_y = 0.0;


_PID_arg_st pos_pid_para;
_PID_val_st pos_pid_value;

_PID_arg_st vec_pid_para;
_PID_val_st vec_pid_value;


float test_filter_x,test_filter_y;
float	position_err_x,position_err_y;

void flow_Decode(const unsigned char* f_buf);


void Flow_Init()
{

	
}

//控制周期20ms
void Flow_Duty(float dT)
{	
	
	pos_pid_para.kp = 0.025f;
	pos_pid_para.ki = 0.004;
	pos_pid_para.kd = 0.0f;

	vec_pid_para.kp = 0.028;
	vec_pid_para.ki = 0.00007;
	vec_pid_para.kd = 0;
	
	if(1 == uart_flag)		//如果接收完成，则转换更新数据，否则使用旧的数据
	{
		static int pos_flag=0;
		uart_flag = 0;					
		flow_Decode(&flow_buffer[0]);		
		
  //速度计算方法：(1.0*flow_dat.y/0.04)*100   1.0为高度，单位m; 0.04为间隔时间,单位s，100是m->cm的转换
		speed_y = flow_dat.y*2500.0f ;  //速度 cm/s  
		speed_x = flow_dat.x*2500.0f ;  //速度 cm/s
		
   //对速度进行限幅		
		speed_y = LIMIT(speed_y,-62.0f,62.0f);  //速度限制在-62cm/s到62cm/s之间
		speed_x = LIMIT(speed_x,-62.0f,62.0f);
				
		//位移计算方法：speed_x*0.04 为0.04s内的位移量，将这个位移量累加，就得到距离开始悬停点的位移量。
		sum_flow_x += speed_x*0.04f; //位移 cm
		sum_flow_y += speed_y*0.04f;
     
	//距离原始悬停位置超过1.0m，这时候应该是用户手打摇杆或漂移比较远了，使得飞机运动较远了，这时候在新的位置悬停
		if(sum_flow_x > POS_CONTROL_LIMIT_MAX || sum_flow_x < POS_CONTROL_LIMIT_MIN) 
			sum_flow_x = 0.0f;
		
		if(sum_flow_y > POS_CONTROL_LIMIT_MAX || sum_flow_y < POS_CONTROL_LIMIT_MIN)
			sum_flow_y = 0.0f;
		pos_flag++;
		
		
	//位置pid控制
		if(2 == pos_flag)   //光流每更新两次执行一次外环，也就是说，位置环的控制周期要大于速度环的控制周期，输出为期望速度
		{
			pos_flag = 0;
			
			pos_pid_out_y = PID_calculate(dT,0.0f,0.0f,sum_flow_y,&pos_pid_para,&pos_pid_value, POS_CALC_OUT_MAX*0.5f);
			pos_pid_out_y = LIMIT(pos_pid_out_y,POS_CALC_OUT_MIN,POS_CALC_OUT_MAX);
			
			pos_pid_out_x = PID_calculate(dT,0.0f,0.0f,sum_flow_x,&pos_pid_para,&pos_pid_value, POS_CALC_OUT_MAX*0.5f);
			pos_pid_out_x = LIMIT(pos_pid_out_x,POS_CALC_OUT_MIN,POS_CALC_OUT_MAX);
		}
	}		
	
//	LPF_1_(2.0f,dT,speed_x,test_filter_x);    //
//	LPF_1_(2.0f,dT,speed_y,test_filter_y);    //pid控制中，如果使用了d，需要选择对速度进行低通滤波，否则飞机会抖动较大
	
	//roll方向速度pid控制，
	exp_rol_flow = PID_calculate(dT,0.0f,pos_pid_out_y*100,speed_y,&vec_pid_para,&vec_pid_value,5.0f);
	exp_rol_flow = -LIMIT(exp_rol_flow,-10.0f,10.0f);//负号是为了匹配控制方向
	
  //pitch方向速度pid控制
	exp_pit_flow = PID_calculate(dT,0.0f,pos_pid_out_x*100,speed_x,&vec_pid_para,&vec_pid_value,5.0f);
	exp_pit_flow = LIMIT(exp_pit_flow,-10.0f,10.0f);
	
	//输出的exp_rol_flow，exp_pit_flow是一个角度值“度”，直接给到姿态控制的角度环，作为一个期望角度(与摇杆给的期望值相当)。
	
	//！！！注意在角度控制中需要如下操作：
	//   if(有摇杆量输入)
	//	 {
	//		exp_rol_flow = 0;
	//		exp_pit_flow = 0;
	//		sum_flow_x = 0;       //有摇杆操作时，定点各个数据清零
	//		sum_flow_y = 0;
				
	//		PID_Value_reset(&pos_pid_value);  //控制量清零
	//		PID_Value_reset(&vec_pid_value);
	//	}
		
	//	except_rol  = roll摇杆期望角度 + exp_rol_flow;   
	//	except_pitch  = pitch摇杆期望角度  + exp_pit_flow;  

}



void flow_Decode(const unsigned char* f_buf)
{	
#ifdef FLOW_INTERFACE_V1_30
                flow_data_frame.pixel_flow_x_integral          = f_buf[0] + (f_buf[1]<<8);
                flow_data_frame.pixel_flow_y_integral          = f_buf[2] + (f_buf[3]<<8);
                flow_data_frame.integration_timespan           = f_buf[4]+ (f_buf[5]<<8) + (f_buf[14]<<16) + (f_buf[15]<<24);
                flow_data_frame.ground_distance                = f_buf[6] + (f_buf[7]<<8);
                flow_data_frame.qual                           = f_buf[8] ;
								flow_data_frame.gyro_temperature               = f_buf[9] ; //实际为新版本中的valid
//                flow_data_frame.valid						               = f_buf[9] ;
#else
		            flow_data_frame.frame_count_since_last_readout = f_buf[0] + (f_buf[1]<<8);
                flow_data_frame.pixel_flow_x_integral          = f_buf[2] + (f_buf[3]<<8);
                flow_data_frame.pixel_flow_y_integral          = f_buf[4] + (f_buf[5]<<8);
                flow_data_frame.gyro_x_rate_integral           = f_buf[6] + (f_buf[7]<<8);
                flow_data_frame.gyro_y_rate_integral           = f_buf[8] + (f_buf[9]<<8);
                flow_data_frame.gyro_z_rate_integral           = f_buf[10]+ (f_buf[11]<<8);
                flow_data_frame.integration_timespan           = f_buf[12]+ (f_buf[13]<<8) + (f_buf[14]<<16) + (f_buf[15]<<24);
                flow_data_frame.sonar_timestamp                = f_buf[16] + (f_buf[17]<<8) + (f_buf[18]<<16) + (f_buf[19]<<24);
                flow_data_frame.ground_distance                = f_buf[20] + (f_buf[21]<<8);
                flow_data_frame.gyro_temperature               = f_buf[22] + (f_buf[23]<<8);
                flow_data_frame.qual                           = f_buf[24] ;
#endif 	
	flow_dat.x = -1.0*flow_data_frame.pixel_flow_x_integral/10000.0f;
	flow_dat.y = -1.0*flow_data_frame.pixel_flow_y_integral/10000.0f;   //为了保留精度，在传输前*10000，所以使用时再/10000，负号是为了匹配传感器方向
  flow_dat.qual =   flow_data_frame.qual;
	flow_dat.dt		=		flow_data_frame.integration_timespan;
	flow_dat.update = 1;
}

void flow_Decode_for_cheap(const unsigned char* f_buf)
{	
                flow_data_frame.pixel_flow_x_integral          = f_buf[0] + (f_buf[1]<<8);
                flow_data_frame.pixel_flow_y_integral          = f_buf[2] + (f_buf[3]<<8);
                flow_data_frame.integration_timespan           = f_buf[4]+ (f_buf[5]<<8) + (f_buf[14]<<16) + (f_buf[15]<<24);
                flow_data_frame.ground_distance                = f_buf[6] + (f_buf[7]<<8);
								flow_data_frame.gyro_temperature               = f_buf[8] ; //实际为新版本中的valid
//                flow_data_frame.valid						               = f_buf[8] ;
                flow_data_frame.qual                           = f_buf[9] ;
	
	flow_dat.x = -1.0*flow_data_frame.pixel_flow_x_integral/10000.0f;
	flow_dat.y = -1.0*flow_data_frame.pixel_flow_y_integral/10000.0f;
  flow_dat.qual =   flow_data_frame.qual;
	flow_dat.dt		=		flow_data_frame.integration_timespan;
	flow_dat.update = 1;
}


void Flow_Get(u8 com_data)
{
	static unsigned char indata,flow_status,i;	

	indata = com_data;		
	
#ifdef FLOW_INTERFACE_V1_30		
		switch(flow_status)
		{
			case 2:
						if(indata == 0x0A)
							flow_status = 3;
						else
							flow_status = 0;
				break;
			case 3:
				    flow_buffer[i++] = indata;
			
				if(12 == i) //28个字节收满,新版本光流12个字节1113
				{
					i=0;
					flow_status = 0;
					uart_flag = 1;	
}
				break;
				
			default:
						if(indata == 0xFE)
							flow_status = 2;
						else
							flow_status = 0;		
						
							uart_flag = 0;	
				break;			
		}	
#else
	switch(flow_status)
		{
			case 2:
						if(indata == 0x1c)
							flow_status = 3;
						else
							flow_status = 0;
				break;
			case 3:
				    flow_buffer[i++] = indata;
			
				if(28 == i) //28个字节收满
				{
					i=0;
					flow_status = 0;
					uart_flag = 1;					
				}
				break;
				
			default:
						if(indata == 0xFE)
							flow_status = 2;
						else
							flow_status = 0;		
						
							uart_flag = 0;					
				break;			
		}		

#endif		
}


void Flow_Get_for_cheap(u8 com_data)
{
	static unsigned char indata,flow_status,i;
	
	indata = com_data;		
//	indata = USART_ReceiveData(USART3);	
		
		switch(flow_status)
		{
			case 2:
						if(indata == 0xA0)
							flow_status = 3;
						else
							flow_status = 0;
				break;
			case 3:
				    flow_buffer[i++] = indata;
			
				if(12 == i) //28个字节收满,新版本光流12个字节1113
				{
					i=0;
					flow_status = 0;
					uart_flag = 1;					
				}
				break;
				
			default:
						if(indata == 0xFE)
							flow_status = 2;
						else
							flow_status = 0;		
						
							uart_flag = 0;					
				break;			
		}		
}

