#include "imu.h"
#include "math.h"

#include "Parameters.h"

#define RtA 			57.324841f	//弧度转角度			
#define AtR    		0.0174533f  //角度转弧度				
#define Acc_Gr 		0.0011963f	//  1/8192*9.8	将加速度的测量出来的值除以对应精度的8192转化为g 乘以9.8 装换为m/s2
#define Gyro_G 		0.0610351f	//  1/16.4  将陀螺仪的测量出来的值除以对应精度的LSB  16.4
#define Gyro_Gr		0.0010653f	//	将Gyro_G*AtR  


#define PI  3.141592654f



#define Kp 1.6f                        // proportional gain governs rate of convergence to accelerometer/magnetometer
//#define Kp 2.0f
#define Ki 0.001f                          // integral gain governs rate of convergence of gyroscope biases
//#define Ki 0.008f
#define halfT 0.001f                   // half the sample period???????

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;    // quaternion elements representing the estimated orientation
float exInt = 0, eyInt = 0, ezInt = 0;    // scaled integral error
void IMUupdate(Axis3i16 *gyr, Axis3i16 *acc, T_float_angle *angle)
{
	float ax = acc->x,ay = acc->y,az = acc->z;
	float gx = gyr->x,gy = gyr->y,gz = gyr->z;
	float mx = magT.x,my = magT.y,mz = magT.z;
  float norm;
	
//	double yaw_mag;//?????????
	double t[3][3]; //??????????
	float q0q3 = q0*q3;
	float q1q2 = q1*q2;
//	int16_t Xr,Yr;
	
  float hx, hy, hz, bx, bz;
  float vx, vy, vz, wx, wy, wz;
  float ex, ey, ez;
	

  // ???????????
  float q0q0 = q0*q0;
  float q0q1 = q0*q1;
  float q0q2 = q0*q2;
//  float q0q3 = q0*q3;
  float q1q1 = q1*q1;
//  float q1q2 = q1*q2;
  float q1q3 = q1*q3;
  float q2q2 = q2*q2;
  float q2q3 = q2*q3;
  float q3q3 = q3*q3;
	
	if(ax*ay*az==0)
 		return;
		
	gx *= Gyro_Gr;
	gy *= Gyro_Gr;
	gz *= Gyro_Gr;
	
//	ax *= Acc_Gr;  乘不乘一样，下面归一化了
//	ay *= Acc_Gr;
//	az *= Acc_Gr;
	//单位化加速度计,意义在于在变更了加速度计的量程之后不需要修改Kp参数,因为这里归一化了
  norm = sqrt(ax*ax + ay*ay + az*az);       //acc?????
  ax = ax /norm;
  ay = ay / norm;
  az = az / norm;

//	norm = sqrt(mx*mx + my*my + mz*mz);          
//        mx = mx / norm;
//        my = my / norm;
//        mz = mz / norm;         
//        
//        // compute reference direction of flux
//        hx = 2*mx*(0.5 - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2);
//        hy = 2*mx*(q1q2 + q0q3) + 2*my*(0.5 - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1);
//        hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.5 - q1q1 - q2q2);         
//        bx = sqrt((hx*hx) + (hy*hy));
//        bz = hz;        
//        
//        // estimated direction of gravity and flux (v and w)
//        vx = 2*(q1q3 - q0q2);
//        vy = 2*(q0q1 + q2q3);
//        vz = q0q0 - q1q1 - q2q2 + q3q3;
//        wx = 2*bx*(0.5 - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);
//        wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
//        wz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5 - q1q1 - q2q2);  
//        
//        // error is sum of cross product between reference direction of fields and direction measured by sensors
//        ex = (ay*vz - az*vy) + (my*wz - mz*wy);
//        ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
//        ez = (ax*vy - ay*vx) + (mx*wy - my*wx);

	//将当前姿态的重力在三个轴上的分量分离出来
	//就是方向余弦旋转矩阵的第三列,注意是地理坐标系(n系)到载体坐标系(b系)的,不要弄反了.如果书上是b系到n系,转置即可
	//始终记着你操纵的是飞机,所以一切的量都要以他的坐标系来做参考,所以坐标需要从地理坐标系上转换到载体坐标系上
  // estimated direction of gravity and flux (v and w)              ?????????/??
  vx = 2*(q1q3 - q0q2);												//????xyz???
  vy = 2*(q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3 ;

	//计算由当前姿态的重力在三个轴上的分量与加速度计测得的重力在三个轴上的分量的差,这里采用三维空间的差积(向量积)方法求差
  // error is sum of cross product between reference direction of fields and direction measured by sensors
  ex = (ay*vz - az*vy) ;                           					 //???????????????
  ey = (az*vx - ax*vz) ;
  ez = (ax*vy - ay*vx) ;

	//积分求误差,关于当前姿态分离出的重力分量与当前加速度计测得的重力分量的差值进行积分消除误差
  exInt = exInt + ex * Ki;								  //???????
  eyInt = eyInt + ey * Ki;
  ezInt = ezInt + ez * Ki;

	//直接应用比例调节，修正陀螺仪的值
  // adjusted gyroscope measurements
  gx = gx + Kp*ex + exInt;					   							//???PI???????,???????
  gy = gy + Kp*ey + eyInt;
  gz = gz + Kp*ez + ezInt;				   							//???gz????????????????,??????????????
	
	//以下为四元数微分方程.将陀螺仪和四元数结合起来，是姿态更新的核心算子
	//计算方法由矩阵运算推导而来
	//	.		1      
	//	q = - * q x Omega    式中左边是四元数的倒数,右边的x是四元数乘法,Omega是陀螺仪的值(即角速度)
	//			2
	//	 .
	//	[q0] 		[0		-wx		-wy		-wz]	[q0]
	//	 .				
	//	[q1]		[wx	  0		  wz		-wy]	[q1]
	//	 .	 =  											* 
	//	[q2]		[wy	 -wz	  0		  wx ]	[q2]
	//	 . 			
	//	[q3]		[wz 	wy	 -wx		0	 ]	[q3]
  // integrate quaternion rate and normalise						   //????????
  q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
  q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
  q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
  q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;
	
	//单位化四元数,意义在于单位化四元数在空间旋转时是不会拉伸的,仅有旋转角度.这类似与线性代数里面的正交变换
  // normalise quaternion
  norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	
  q0 = q0 / norm;
  q1 = q1 / norm;
  q2 = q2 / norm;
  q3 = q3 / norm;
	
	
	q0q0=q0*q0;
  q0q1=q0*q1;
  q0q2=q0*q2;
  q0q3=q0*q3;
  q1q1=q1*q1;
  q1q2=q1*q2;
  q1q3=q1*q3;
  q2q2=q2*q2;
  q2q3=q2*q3;
  q3q3=q3*q3;

  t[1][2]=2.0*(q2q3+q0q1); //y 
  t[2][2]=q0q0-q1q1-q2q2+q3q3; //z
  t[0][2]=2.0*(q1q3-q0q2); 	//x
  t[0][1]=2.0*(q1q2+q0q3);
  t[0][0]=q0q0+q1q1-q2q2-q3q3;  //????????
	
	//四元数到欧拉角转换
	//angle->yaw += gyr->z*Gyro_G*0.002f;
	angle->yaw = -atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2*q2 - 2 * q3 * q3 + 1)* 57.3; //gyr->Z*Gyro_G*0.002f;
//	angle->pit = -asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3 ;//- AngleOffset_Pit; // pitch
//	angle->rol = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3 ;//- AngleOffset_Rol; // roll
//	
	//angle->yaw = -atan2(t[0][1],t[0][0])*57.3; 
//	
	angle->pit = -asin(t[0][2])* 57.3 ;//- AngleOffset_Pit; // pitch
	angle->rol = atan2(t[1][2],t[2][2])* 57.3;
	
	
		/*地磁倾角补偿 采用三角函数泰勒展开式  节省运行时间*/
//	Xr = magT.x * (1 - angle->pit * angle->pit * 0.5) + magT.y * (-angle->pit + angle->pit*angle->pit*angle->pit/6)*(-angle->rol + angle->rol*angle->rol*angle->rol/6) - magT.z * (1-angle->rol*angle->rol*0.5) * (-angle->pit + angle->pit*angle->pit*angle->pit/6);
//	Yr = magT.y * (1 - angle->rol * angle->rol * 0.5) + magT.z * (-angle->rol + angle->rol*angle->rol*angle->rol/6);
//	angle->yaw= atan2((double)Yr,(double)Xr) * RtA; // yaw
//	  
//		angle->yaw = atan2(magT.y ,magT.x) * RtA; // yaw
	
//	norm = sqrt(magT.x*magT.x + magT.y*magT.y + magT.z*magT.z);
//  magT.x = magT.x / norm;
//  magT.y = magT.y / norm;
//  magT.z = magT.z / norm;

//  yaw_mag = atan2( t[0][2]*magT.z - t[2][2]*magT.x , t[2][2]*magT.y - t[1][2]*magT.z)+PI/2+0.095993108;//??:?? (0~2*PI),????????????,??????5.5�??
//  //heading = _atan2f(EstG.V.X * EstM.V.Z - EstG.V.Z * EstM.V.X, EstG.V.Z * EstM.V.Y - EstG.V.Y * EstM.V.Z) / 10;??MWC????????
//  //?????x??????????0�,?????????,?????????,???0~360�

//  if(yaw_mag<0)
//    yaw_mag+=(2*PI);
//  if(yaw_mag>2*PI)
//    yaw_mag-=(2*PI); //??????????? 0~2PI

//  if(fabs(yaw_mag-angle->yaw) < PI) 
//  {
//     angle->yaw += (-1)*gz*0.002 + (angle->yaw)*0.002;  
//  }

//  if(fabs(yaw_mag-angle->yaw) >= PI) // ???????2PI????,????????PI??????????????
//  {
//     angle->yaw += (-1)*gz*0.002 - (yaw_mag-angle->yaw)*0.002;  
//  }  
//  //?????????????,gz??????,???2ms????IMUupdata(),??gz*0.002??2ms?????,???????????
//  //???????????????????????????,??????0.002(???)????????????????
//  //????,??????????????,????????????????????????,???????????
//  //??????,?????????????????????????????????????????(?�15�???�3�)
//  
//	angle->yaw = angle->yaw* 57.3;
	
}
