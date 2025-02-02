#ifndef _ONBOARD_MAH01_H_
#define _ONBOARD_MAH01_H_

#include <Encoder.h>


//Motor param config
	#define MotorType					DC_Motor

// define your robot' specs here
	#define MOTOR_MAX_PWM 255 // motor's maximum RPM
	#define MAX_RPM 330 // motor's maximum RPM
	#define COUNTS_PER_REV 2300 // wheel encoder's no of ticks per rev
	#define WHEEL_DIAMETER 0.068 // wheel's diameter in meters
	#define PWM_BITS 8 // PWM Resolution of the microcontroller
	#define BASE_WIDTH 0.26 // width of the plate you are using
	
//PID parmeter
	#define K_P 2.0 // P constant
	#define K_I 0.2 // I constant
	#define K_D 0.2 // D constant
	
struct port_t
{
   int Dir_A;
   int Dir_B;
   int PWM_Pin;
   int Encoder_A;
   int Encoder_B;
};

//====================
//====Mecanum car=====
//                      
//  A\\-(X+)-//(B)
//    |      |
//   (Y-)   (Y+) Right
//    |      |
//  C//-(X-)-\\(D)
//
//===================
// Define motor port
#define PORT_LF   0
#define PORT_RF   1
#define PORT_LR   2
#define PORT_RR   3

#ifdef HIGH_POWER_DRIVE
port_t port[4] =
{
// A    B   PWM  EN_A  EN_B
  {A4,  A5,   4,  18,  31},   // A-- Left  Front(LF)
  {42,  43,   6,  19,  38},   // B-- Right Front(RF)
  {36,  37,   7,   2,  A1},   // C-- Left  Rear (LR)
  {34,  35,  11,   3,  49},   // D-- Right Rear (RR)
};
#else
port_t port[4] =
{
// A    B   PWM  EN_A  EN_B
  {35,  34,  12,  18,  31},   // A-- Right Rear (RR)
  {36,  37,   8,  19,  38},   // B-- Left  Rear (LR)
  {42,  43,   9,   3,  49},   // C-- Right Front(RF)
  {A5,  A4,   5,   2,  A1},   // D-- Left  Front(LF)
};
#endif

class DC_Motor
{
private:
    int Dir_A, Dir_B;
    int PWM_Pin;
    int Encoder_A, Encoder_B;
    Encoder *_encoder;
    int dir;
    float kp, ki, kd;
	int Port;
    long prev_encoder_ticks_, prev_update_time_;
    int Last_tar, Bias, Last_bias, Pwm, Last_Pwm, Out_Pwm;
    int rpm;

public:
    DC_Motor(int Motor_Port)    
    {
        Port = Motor_Port;
        Dir_A = port[Port].Dir_A;
        Dir_B = port[Port].Dir_B;
        PWM_Pin = port[Port].PWM_Pin;
        Encoder_A = port[Port].Encoder_A;
        Encoder_B = port[Port].Encoder_B;
    }
    int Init(int _dir)
    {
        pinMode(Dir_A, OUTPUT);
        pinMode(Dir_B, OUTPUT);
        pinMode(PWM_Pin, OUTPUT);
        _encoder = new Encoder(Encoder_A, Encoder_B);
        kp = K_P;
        ki = K_I;
        kd = K_D;
        dir = _dir;
    }
	
	void PrintPortConfig()
	{
        Serial.print("Motor Prot : ");
        Serial.print(Port);   
        Serial.print(", Dir : ");
        Serial.print(Dir_A);
        Serial.print(", ");
        Serial.print(Dir_B);
        Serial.print("\t, PWM : ");
        Serial.print(PWM_Pin);
        Serial.print(", Encoder : ");
        Serial.print(Encoder_A);
        Serial.print(", ");
        Serial.println(Encoder_B);
	}
	
    int setSpd(int spd)
    {
        spd = dir?(-spd):(spd);
        if(spd > 0)
        {
            digitalWrite(Dir_A, HIGH);
            digitalWrite(Dir_B, LOW);
        }
        else if(spd < 0)
        {
            digitalWrite(Dir_A, LOW);
            digitalWrite(Dir_B, HIGH);
        }
        else
        {
            digitalWrite(Dir_A, LOW);
            digitalWrite(Dir_B, LOW);
        }
		analogWrite(PWM_Pin, abs(spd));  
    }

    long getEncoderPosition()
    {
        long position = _encoder->read();
        return dir ? -position : position;
    }
	
    void clrEncoderPosition()
    {
        _encoder->write(0);
    }
	
	int getMotorRPM()
	{
		return dir ? -rpm : rpm;
	}
	
    void updateSpd()
    {
//      this function calculates the motor's RPM based on encoder ticks and delta time
//      convert the time from milliseconds to minutes
      unsigned long current_time = millis();
      unsigned long dt = current_time - prev_update_time_;
      double dtm = (double)dt / 60000;
     //   double dtm = 0.000167;
        double delta_ticks = getEncoderPosition() - prev_encoder_ticks_;

        rpm = (delta_ticks / COUNTS_PER_REV) / dtm;    //calculate wheel's speed (RPM)
      prev_update_time_ = current_time;
        prev_encoder_ticks_ = getEncoderPosition();
    }

    //PID
    void Incremental_PID(int target)
    { 
        updateSpd();
        if(Last_tar > target)       
          Last_tar-=2;
        else if(Last_tar < target)
          Last_tar+=2;
        Bias = rpm - Last_tar;          
        Pwm += kp * (Bias - Last_bias) + ki * Bias;       
        if(Pwm > MOTOR_MAX_PWM)  Pwm = MOTOR_MAX_PWM;   
        if(Pwm < -MOTOR_MAX_PWM) Pwm = -MOTOR_MAX_PWM;
        Last_bias=Bias;                             
        Out_Pwm *= 0.7;                             
        Out_Pwm += Last_Pwm * 0.3;
        Last_Pwm = Pwm;
        if(Out_Pwm < 6 && Out_Pwm > -6) Out_Pwm = 0;
        setSpd(Out_Pwm);
    }

    void Update_PID(float _kp, float _ki, float _kd)
    {
        kp = _kp;
        ki = _ki;
        kd = _kd;
    }
};

#endif
