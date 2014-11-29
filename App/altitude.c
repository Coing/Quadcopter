#include "stm32f10x.h"
#include "altitude.h"

// Altitude hold variables
PidObject ult_pid; // Used for altitute hold mode. I gets reset when the bat status changes
//u8 setAltHold = 0;      // Hover mode has just been activated
u8 altHold = 0;
// Altitude hold & sonar Params
static float altHoldKp              = 100.0;  // PID gain constants, used everytime we reinitialise the PID controller
static float altHoldKi              = 0.0;
static float altHoldKd              = 0.0;

static float altHoldTarget          = -1;    // Target altitude

static uint16_t altHoldMinThrust    = 00000; // minimum hover thrust - not used yet
static uint16_t altHoldMaxThrust    = 500; //max altitude hold thrust 最大400吧



static float rang = 0;		//距离
static float speed = 0;  //速度

// sonar/ Altitude hold stuff  几hz好呢, 不行的话就和 attitude一样吧 500hz  不过数据更新慢啊，不知道
#define ALTHOLD_UPDATE_FREQ  				100 	 // 100hz
#define ALTHOLD_UPDATE_DT  					(float)(1.0 / (ALTHOLD_UPDATE_FREQ))   // 1/100hz

#define ALTHOLD_DATE_UPDATE_DT      0.06f   //60ms

//multiple axis sliding windows filter
#undef   FILTER_WINDOWS
#define  FILTER_WINDOWS    5	 //1s    
#define  FILTER_CHANNEL		2		 // 
/*
			5*x1 + 4*x2
like	-----------
					9
*/
static void sliding_windows_filter(float in[FILTER_CHANNEL], float out[FILTER_CHANNEL])
{
	static float buf[FILTER_WINDOWS][FILTER_CHANNEL];
	static int buf_pointer = 0;
	int weight = FILTER_WINDOWS;
	int count = FILTER_WINDOWS;
	int ch_count = FILTER_CHANNEL;
	int index = buf_pointer;
	int total_weight = 0;
	float integral[FILTER_CHANNEL] = {0};
	
	//add the newest date to the buffer
	ch_count = FILTER_CHANNEL;
	while(ch_count>0)
	{
		ch_count--;
		buf[index][ch_count] = in[ch_count];
	}


	//filter
	while(count--)
	{
		ch_count = FILTER_CHANNEL;
		while(ch_count>0)
		{
			ch_count--;
			integral[ch_count] += buf[index][ch_count] * weight;
		}
		
		total_weight += weight; //the newest data have the most significant weight
		weight--;				
				
		index++;
		if(index >= FILTER_WINDOWS)
			index = 0;
	}
	buf_pointer ++;
	if(buf_pointer >= FILTER_WINDOWS)
		buf_pointer = 0;
	
	//update the newest date
	ch_count = FILTER_CHANNEL;
	while(ch_count>0)
	{
		ch_count--;
		out[ch_count] = (integral[ch_count] / total_weight);
	}
}

float rang_bef = 0;
float in[2];
float out[2];



/* 超声波测距输出数据 */
_ult_data ult_data;

//void filter_hcsr( uint16_t alt)
void filter_hcsr( float alt)
{
	//get lenght  
	//in[0] = (float)alt/100.f ;
	//统一为 meter  unit
	in[0] = (float)alt/10000.f ;
	//get speed
	in[1] = (in[0] - rang_bef)/ ALTHOLD_DATE_UPDATE_DT;

			//filter
	sliding_windows_filter(in, out);

	//get data after filter
	rang = out[0];
	speed = out[1];
	
	//mark last rang
	rang_bef = rang;
	
	ult_data.distance = rang;	
	ult_data.speed = speed;
	ult_data.altitude = ult_data.distance;
				
}




static float exp_throttle = -500;
static float ult_lock_height = 0;
static float airpress_lock_height = 0;
static float steady_throttle = -500;

float stabilizerAltHoldUpdate(u8 mode, int16_t THROTTLE)
{
	if(mode == 1)
	{
		//if ultrasonic data is good
			if( (ult_data.altitude < 2.8f)&&(ult_data.altitude > 0.0f))
			{
				float throttle_delta = 0;
				
				//use acc intergral
//				acc_altitude_intergral(ENABLE);
				
				throttle_delta = ult_pid_computer(ult_lock_height, ult_data.altitude, ult_data.speed);
				//定在0.7m。。。
			//	throttle_delta = ult_pid_computer(0.7, ult_data.altitude, ult_data.speed);
		
				exp_throttle = steady_throttle+throttle_delta;
				
//				if(exp_throttle > 200)
//					exp_throttle = 200;
//				if(exp_throttle < -500)
//					exp_throttle = -500;
				
				return (float)exp_throttle;
			}
		
	}
	else
	{
					//acc altitude disable
			//acc_altitude_intergral(DISABLE);
//			acc_altitude_intergral(ENABLE);
			
			//update current altitude for hold altitude mode
			ult_lock_height = ult_data.altitude;
//			airpress_lock_height = acc_alt;//air_info.altitude;
			
			//update exp_throttle from current ppm input throttle
			exp_throttle = THROTTLE;
//			
//			//mark steady throttle from now ppm 
			steady_throttle = exp_throttle;
		
	}
	
	return (float)exp_throttle;
}



float ult_pid_computer(float exp_height, float current_height,  float current_speed)
{
	float output = 0;
	float height_err = (exp_height - current_height);
	float delta = 0;
	
	#define HEIGHT_LIMIT 1 //meters
	if(height_err > HEIGHT_LIMIT) height_err = HEIGHT_LIMIT;
	if(height_err < -HEIGHT_LIMIT) height_err = -HEIGHT_LIMIT;
	
	#define SPEED_LIMIT 1 //meters
	if(current_speed > SPEED_LIMIT) current_speed = SPEED_LIMIT;
	if(current_speed < -SPEED_LIMIT) current_speed = -SPEED_LIMIT;
	
	//the litl value is the result of the distance from current waypoint to taget waypoint 
	delta = (height_err * ult_pid.kp - current_speed * ult_pid.kd);	
	
	if(delta > altHoldMaxThrust)	 delta = altHoldMaxThrust;
	if(delta < -altHoldMaxThrust) delta = -altHoldMaxThrust;
		
	output = delta;
	
	return output;
}

void AltControlInit()
{
//定高初始化，当时在初始化，进入定高模式的时候, 要弄进去吗
	pidInit(&ult_pid, 0, altHoldKp, altHoldKi, altHoldKd, ALTHOLD_UPDATE_DT);

}
