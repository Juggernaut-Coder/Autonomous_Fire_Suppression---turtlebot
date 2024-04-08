// Purpose: Motor Control for ECED3901 robots
// DO NOT USE TIMER 0, used for delay and ROS functions
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>
#include <PID_v1.h>

#define wheel_radius 0.0325
#define wheel_axis 0.200
#define tick_to_rpm 1.9073486328125
#define pi 3.14159265359


// Global variables
ros::NodeHandle  nh;       //ROS node handle for pubs/subs.

// Motors
double omega_right_setpoint = 0.0;   //Right motor RPM setpoint
double omega_left_setpoint = 0.0;    //Left motor RPM setpoint
double omega_right_setpoint_abs = 0.0;   //Right motor RPM setpoint
double omega_left_setpoint_abs = 0.0;    //Left motor RPM setpoint

// Encoders
boolean dir_right_measured;  //Right motor direction from encoders
boolean dir_left_measured;   //Left motor direction from encoders
double omega_right_measured;  //Right motor RPM from encoders
double omega_left_measured;   //Left motor RPM from encoders
double omega_right_measured_abs;  //Right motor RPM from encoders
double omega_left_measured_abs;   //Left motor RPM from encoders
int right_temp_ticks;
int left_temp_ticks;
int32_t right_ticks = 0;
int32_t left_ticks = 0;
unsigned int past_PINB = 0x00;
unsigned int past_PIND = 0x00;

// PID
double Kp=2, Ki=10, Kd=0.02;
double right_pwm, left_pwm;
boolean rightResult, leftResult;
PID rightPID(&omega_right_measured_abs, &right_pwm, &omega_right_setpoint_abs, Kp, Ki, Kd, DIRECT);
PID leftPID(&omega_left_measured_abs, &left_pwm, &omega_left_setpoint_abs, Kp, Ki, Kd, DIRECT);

// Subscriber 
// ROS message handling fxns/setups
void messageCb( const geometry_msgs::Twist& msg)
{   
    omega_right_setpoint = ((msg.linear.x/wheel_radius) + ((msg.angular.z*wheel_axis)/(2*wheel_radius)))*30/pi;
    omega_left_setpoint = ((msg.linear.x/wheel_radius) - ((msg.angular.z*wheel_axis)/(2*wheel_radius)))*30/pi;
    omega_right_setpoint_abs = abs(omega_right_setpoint);
    omega_left_setpoint_abs = abs(omega_left_setpoint);    
}
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageCb );


void LED_blink(const std_msgs::Int32& msg) {
  digitalWrite(13,HIGH);
  delay(3000);
  digitalWrite(13,LOW); 
}
ros::Subscriber<std_msgs::Int32> sub1("blink", &LED_blink);

 //Publisher
std_msgs::Int32 rwheel_msg, lwheel_msg;        //Ticks
ros::Publisher rticks("rwheel", &rwheel_msg);
ros::Publisher lticks("lwheel", &lwheel_msg);


//Function declarations
void MCU_init();
void set_motor_right(int input_pwm);
void set_motor_left(int input_pwm);



//Interupt Service Routines
ISR(PCINT0_vect)
{
    if( (PINB & (1<<PB0)) && !(past_PINB & (1<<PB0)) ) //trigger on rising-edge only
    {
      if(PINB & (1<<PB1)){right_temp_ticks--;right_ticks--;}  //reverse
      else {right_temp_ticks++;right_ticks++;}                //forward
    }   
    past_PINB = PINB;
}

ISR(PCINT2_vect)
{
    if( (PIND & (1<<PD5)) && !(past_PIND & (1<<PD5)) ) //trigger on rising-edge only
    {
      if(PIND & (1<<PD6)){left_temp_ticks--;left_ticks--;}  //reverse
      else {left_temp_ticks++;left_ticks++;}                //forward
    }  
    past_PIND = PIND;
}

ISR(TIMER1_OVF_vect)
{
     //Calculate the RPM from the ticks over a fixed time window
     //16 MHz crystal / 8 = 2 MHz / 65536 counts = 30.52 Hz update rate
     //8 encoder ticks x 120 gear = 960 ticks per revolution, 1 RPM = 16 ticks/sec
     //2,000,000 / 1,048,576 = 1.9073... tick-to-rpm conv.
     omega_right_measured = (double)right_temp_ticks * tick_to_rpm;
     omega_left_measured = (double)left_temp_ticks * tick_to_rpm;
     omega_right_measured_abs = abs(omega_right_measured);
     omega_left_measured_abs = abs(omega_left_measured);
     //Reset temp tick counters for next measurement window     
     right_temp_ticks = 0;
     left_temp_ticks = 0;     
}

void setup()
{
        MCU_init();
        nh.initNode();
        nh.subscribe(sub); 
  
        nh.advertise(rticks);
        nh.advertise(lticks);
     
        rightPID.SetMode(AUTOMATIC);//PID is set to automatic mode
        rightPID.SetSampleTime(100);//Set PID sampling frequency is 100ms
        leftPID.SetMode(AUTOMATIC);//PID is set to automatic mode
        leftPID.SetSampleTime(100);//Set PID sampling frequency is 100ms
        pinMode(13,OUTPUT);
        nh.subscribe(sub1);        
}


void loop()
{
        rightResult=rightPID.Compute();//PID conversion is complete and returns 1
        leftResult=leftPID.Compute();//PID conversion is complete and returns 1
        
        if(rightResult)
        {        
          if(omega_right_setpoint<0) {set_motor_right(-1*(int)right_pwm);}
          else {set_motor_right((int)right_pwm);}
          
          rwheel_msg.data = right_ticks;
          rticks.publish ( &rwheel_msg );                      
        }        
        
        if(leftResult)
        {
          if(omega_left_setpoint<0) {set_motor_left(-1*(int)left_pwm);}            
          else {set_motor_left((int)left_pwm);}
          
          lwheel_msg.data = left_ticks;
          lticks.publish ( &lwheel_msg );
        }      
        nh.spinOnce();
        delay(1);
}

//Function implementations --------------------------------------------------------------------------
void MCU_init()
{
  //Set pin directions as inputs or outputs
        DDRB |= (1<<PB3) | (1<<PB4);            //PB3 = Output, Right PWM channel OC2A. PB4 = Output, Right direction.
        DDRD |= (1<<PD3) | (1<<PD4);            //PD3 = Output, Left PWM channel OC2B. PD4 = Output, Left direction.
        DDRB &= ~( (1<<PB0) | (1<<PB1) );       //PB0 = Input, Right Encoder A ch.  PB1 = Input, Right Encoder B ch.
        DDRD &= ~( (1<<PD5) | (1<<PD6) );       //PD5 = Input, Left Encoder A ch.  PD6 = Input, Left Encoder B ch.
        
  //Set the timer registers to PWM as desired  
	TCNT2 = 0x00;				//Timer2 start at zero
	TCCR2A = 0x03;				//Timer2 Control Register A
                                                  //0xA3        //A & B clear on comp. non-inverting both channels 
								//Mode 3: FastPWM + MAX limit
	TCCR2B = 0x02;				//Timer2 Control Register B, 
                                                  //0x00        //Stop Timer2, no clock source
                                                  //0x02        //Set Timer2 clock source to clkI/O  / 8 prescaler
                                                                //16 MHz crystal / 8, 2 MHz / 256 counts = 7.81 kHz PWM
	OCR2A = 0;				//Timer2 Output Compare Register A, 8-bit 
	OCR2B = 0;				//Timer2 Output Compare Register B, 8-bit
   
  //Set motor pin outputs to gnd or logic 0, 0 = off/forward, 1 = on/reverse
        PORTB &= ~((1 << PB3)|(1 << PB4));
        PORTD &= ~((1 << PD3)|(1 << PD4)); 
  
  //Set Timer 1 to monitor Encoders, interupt service routine uses
        TCNT1 = 0x00;				//Timer1 start at zero
	TCCR1A = 0x00;				//Timer1 Control Register A, normal mode                                                
	TCCR1B = 0x02;  			//Timer1 Control Register B, 
                                                  //0x00        //Stop Timer1, no clock source
                                                  //0x02        //Set Timer1 clock source to clkI/O  / 8 prescaler  
                                                                //16 MHz crystal / 8 = 2 MHz / 65536 counts = 30.52 Hz update rate
                                                                //16 encoder ticks x 120 gear = 1920 ticks, at 2 RPM, 0.033 RPS
                                                                //1/0.033 ~ 30Hz, therefore, lowest speed is 2 RPM
                                                               
  //Set Masks for Pin Change Interrupts
        cli();                                  //Disable interrupts while changing registers
        PCICR |= (1<<PCIE0) | (1<< PCIE2);      //Pin Change Interrupt Control Register
                                                //PB0 is PCINT0 on PCIE0, PD5 is PCINT21 on PCIE2
        PCMSK0 = (1<<PCINT0);                   //Pin Change Mask Register, PB0 only interrupt 
        PCMSK2 = (1<<PCINT21);                  //Pin Change Mask Register, PD5 only interrupt
        TIMSK1 = (1<<TOIE1);                    //Enable timer1 overflow interrupt (TOIE1)
        sei();                                  //Enable interrupts after config.       
               
}


void set_motor_right(int input_pwm)
{
        //Set direction from PWM input -255 to 255
        if (input_pwm < 0)
        {
            PORTB |= (1 << PB4);                //Set PB4 to 1, reverse
            input_pwm = -1*input_pwm;           //make positive for setting magnitude below
        }
        else 
        {
            PORTB &= ~(1 << PB4);               //Set PB4 to 0, forward           
        }            

        //Bound PWM to 8-bits
        if (input_pwm > 255) {input_pwm = 255;}  
        
        //Set PWM to counter compare regs.        
        //Stop is special case of PWM=0
        if (input_pwm == 0)  	
  	{			
  		TCCR2A &= ~((1 << COM2A1)|(1 << COM2A0)); //Clear bits to disconnect OC2A 
                OCR2A = 0;		
  		PORTB &= ~((1 << PB3));	        //Set Outputs to 0, B3 = 0;
  		//PORTB |= ((1 << PB3));	//Set Outputs to 1, B3 = 1;
  	}   
        else
        {
                TCCR2A |= (1 << COM2A1);      //Set bits to connect OC2A to pin B3 and clear on compare match
                TCCR2A &= ~(1 << COM2A0);     //Set bits to connect OC2A to pin B3 and clear on compare match    
                OCR2A = input_pwm;            //send pwm to compare reg
        }    
}


void set_motor_left(int input_pwm)
{
        //Set direction from PWM input -255 to 255
        if (input_pwm < 0)
        {
            PORTD |= (1 << PD4);                //Set PD4 to 1, reverse
            input_pwm = -1*input_pwm;           //make positive for setting magnitude below
        }
        else 
        {
            PORTD &= ~(1 << PD4);               //Set PD4 to 0, forward           
        }            

        //Bound PWM to 8-bitds
        if (input_pwm > 255) {input_pwm = 255;}  
        
        //Set PWM to counter compare regs.        
        //Stop is special case of PWM=0
        if (input_pwm == 0)  	
  	{			
  		TCCR2A &= ~((1 << COM2B1)|(1 << COM2B0)); //Clear bits to disconnect OC2B 
                OCR2B = 0;		
  		PORTD &= ~((1 << PD3));	        //Set Outputs to 0, D3 = 0;
  		//PORTD |= ((1 << PD3));	//Set Outputs to 1, D3 = 1;
  	}   
        else
        {
                TCCR2A |= (1 << COM2B1);      //Set bits to connect OC2B to pin D6 and clear on compare match
                TCCR2A &= ~(1 << COM2B0);     //Set bits to connect OC2B to pin D6 and clear on compare match    
                OCR2B = input_pwm;            //send pwm to compare reg
        }  
}
