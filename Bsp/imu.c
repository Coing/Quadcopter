#include "imu.h"
#include "math.h"

#include "Parameters.h"

#define RtA 			57.324841f	//»¡¶È×ª½Ç¶È			
#define AtR    		0.0174533f  //½Ç¶È×ª»¡¶È				
#define Acc_Gr 		0.0011963f	//  1/8192*9.8	½«¼ÓËÙ¶ÈµÄ²âÁ¿³öÀ´µÄÖµ³ýÒÔ¶ÔÓ¦¾«¶ÈµÄ8192×ª»¯Îªg ³ËÒÔ9.8 ×°»»Îªm/s2
#define Gyro_G 		0.0610351f	//  1/16.4  ½«ÍÓÂÝÒÇµÄ²âÁ¿³öÀ´µÄÖµ³ýÒÔ¶ÔÓ¦¾«¶ÈµÄLSB  16.4
#define Gyro_Gr		0.0010653f	//	½«Gyro_G*AtR  


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
	
//	ax *= Acc_Gr;  ³Ë²»³ËÒ»Ñù£¬ÏÂÃæ¹éÒ»»¯ÁË
//	ay *= Acc_Gr;
//	az *= Acc_Gr;
	//µ¥Î»»¯¼ÓËÙ¶È¼Æ,ÒâÒåÔÚÓÚÔÚ±ä¸üÁË¼ÓËÙ¶È¼ÆµÄÁ¿³ÌÖ®ºó²»ÐèÒªÐÞ¸ÄKp²ÎÊý,ÒòÎªÕâÀï¹éÒ»»¯ÁË
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

	//½«µ±Ç°×ËÌ¬µÄÖØÁ¦ÔÚÈý¸öÖáÉÏµÄ·ÖÁ¿·ÖÀë³öÀ´
	//¾ÍÊÇ·½ÏòÓàÏÒÐý×ª¾ØÕóµÄµÚÈýÁÐ,×¢ÒâÊÇµØÀí×ø±êÏµ(nÏµ)µ½ÔØÌå×ø±êÏµ(bÏµ)µÄ,²»ÒªÅª·´ÁË.Èç¹ûÊéÉÏÊÇbÏµµ½nÏµ,×ªÖÃ¼´¿É
	//Ê¼ÖÕ¼Ç×ÅÄã²Ù×ÝµÄÊÇ·É»ú,ËùÒÔÒ»ÇÐµÄÁ¿¶¼ÒªÒÔËûµÄ×ø±êÏµÀ´×ö²Î¿¼,ËùÒÔ×ø±êÐèÒª´ÓµØÀí×ø±êÏµÉÏ×ª»»µ½ÔØÌå×ø±êÏµÉÏ
  // estimated direction of gravity and flux (v and w)              ?????????/??
  vx = 2*(q1q3 - q0q2);												//????xyz???
  vy = 2*(q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3 ;

	//¼ÆËãÓÉµ±Ç°×ËÌ¬µÄÖØÁ¦ÔÚÈý¸öÖáÉÏµÄ·ÖÁ¿Óë¼ÓËÙ¶È¼Æ²âµÃµÄÖØÁ¦ÔÚÈý¸öÖáÉÏµÄ·ÖÁ¿µÄ²î,ÕâÀï²ÉÓÃÈýÎ¬¿Õ¼äµÄ²î»ý(ÏòÁ¿»ý)·½·¨Çó²î
  // error is sum of cross product between reference direction of fields and direction measured by sensors
  ex = (ay*vz - az*vy) ;                           					 //???????????????
  ey = (az*vx - ax*vz) ;
  ez = (ax*vy - ay*vx) ;

	//»ý·ÖÇóÎó²î,¹ØÓÚµ±Ç°×ËÌ¬·ÖÀë³öµÄÖØÁ¦·ÖÁ¿Óëµ±Ç°¼ÓËÙ¶È¼Æ²âµÃµÄÖØÁ¦·ÖÁ¿µÄ²îÖµ½øÐÐ»ý·ÖÏû³ýÎó²î
  exInt = exInt + ex * Ki;								  //???????
  eyInt = eyInt + ey * Ki;
  ezInt = ezInt + ez * Ki;

	//Ö±½ÓÓ¦ÓÃ±ÈÀýµ÷½Ú£¬ÐÞÕýÍÓÂÝÒÇµÄÖµ
  // adjusted gyroscope measurements
  gx = gx + Kp*ex + exInt;					   							//???PI???????,???????
  gy = gy + Kp*ey + eyInt;
  gz = gz + Kp*ez + ezInt;				   							//???gz????????????????,??????????????
	
	//ÒÔÏÂÎªËÄÔªÊýÎ¢·Ö·½³Ì.½«ÍÓÂÝÒÇºÍËÄÔªÊý½áºÏÆðÀ´£¬ÊÇ×ËÌ¬¸üÐÂµÄºËÐÄËã×Ó
	//¼ÆËã·½·¨ÓÉ¾ØÕóÔËËãÍÆµ¼¶øÀ´
	//	.		1      
	//	q = - * q x Omega    Ê½ÖÐ×ó±ßÊÇËÄÔªÊýµÄµ¹Êý,ÓÒ±ßµÄxÊÇËÄÔªÊý³Ë·¨,OmegaÊÇÍÓÂÝÒÇµÄÖµ(¼´½ÇËÙ¶È)
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
	
	//µ¥Î»»¯ËÄÔªÊý,ÒâÒåÔÚÓÚµ¥Î»»¯ËÄÔªÊýÔÚ¿Õ¼äÐý×ªÊ±ÊÇ²»»áÀ­ÉìµÄ,½öÓÐÐý×ª½Ç¶È.ÕâÀàËÆÓëÏßÐÔ´úÊýÀïÃæµÄÕý½»±ä»»
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
	
	//ËÄÔªÊýµ½Å·À­½Ç×ª»»
	//angle->yaw += gyr->z*Gyro_G*0.002f;
	angle->yaw = -atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2*q2 - 2 * q3 * q3 + 1)* 57.3; //gyr->Z*Gyro_G*0.002f;
//	angle->pit = -asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3 ;//- AngleOffset_Pit; // pitch
//	angle->rol = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3 ;//- AngleOffset_Rol; // roll
//	
	//angle->yaw = -atan2(t[0][1],t[0][0])*57.3; 
//	
	angle->pit = -asin(t[0][2])* 57.3 ;//- AngleOffset_Pit; // pitch
	angle->rol = atan2(t[1][2],t[2][2])* 57.3;
	
	
		/*µØ´ÅÇã½Ç²¹³¥ ²ÉÓÃÈý½Çº¯ÊýÌ©ÀÕÕ¹¿ªÊ½  ½ÚÊ¡ÔËÐÐÊ±¼ä*/
//	Xr = magT.x * (1 - angle->pit * angle->pit * 0.5) + magT.y * (-angle->pit + angle->pit*angle->pit*angle->pit/6)*(-angle->rol + angle->rol*angle->rol*angle->rol/6) - magT.z * (1-angle->rol*angle->rol*0.5) * (-angle->pit + angle->pit*angle->pit*angle->pit/6);
//	Yr = magT.y * (1 - angle->rol * angle->rol * 0.5) + magT.z * (-angle->rol + angle->rol*angle->rol*angle->rol/6);
//	angle->yaw= atan2((double)Yr,(double)Xr) * RtA; // yaw
//	  
//		angle->yaw = atan2(magT.y ,magT.x) * RtA; // yaw
	
//	norm = sqrt(magT.x*magT.x + magT.y*magT.y + magT.z*magT.z);
//  magT.x = magT.x / norm;
//  magT.y = magT.y / norm;
//  magT.z = magT.z / norm;

//  yaw_mag = atan2( t[0][2]*magT.z - t[2][2]*magT.x , t[2][2]*magT.y - t[1][2]*magT.z)+PI/2+0.095993108;//??:?? (0~2*PI),????????????,??????5.5°??
//  //heading = _atan2f(EstG.V.X * EstM.V.Z - EstG.V.Z * EstM.V.X, EstG.V.Z * EstM.V.Y - EstG.V.Y * EstM.V.Z) / 10;??MWC????????
//  //?????x??????????0°,?????????,?????????,???0~360°

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
//  //??????,?????????????????????????????????????????(?±15°???±3°)
//  
//	angle->yaw = angle->yaw* 57.3;
	
}
