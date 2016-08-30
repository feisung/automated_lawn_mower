/******************************************************************************
*  Code for Automated garden sweeper         *Code written by:  Fei Manheche  *
*  For: UEL final year project       *Project start date: end of Sept. 2010   *
*              Servo motor and Ultrasonic transducer control 1                *
*******************************************************************************/
#include "math.h"             //Calling for the math library
#include <Servo.h>            //Calling for the servo library

//Constants and functions for the Ultrasonic transducer and its function
int RxPin = 2;              // the SRF05's echo pin
int TxPin = 3;              // the SRF05's init pinint ultraSoundpin = 6;
unsigned long pulseTime = 0;  // variable for reading the pulse
const int Nreadings = 4;   // set a variable for the number of readings to take
unsigned long distance = 0;   // variable for storing distance
int index = 0;                // the index of the current reading
int total = 0;                // the total of all readings
int average = 0;              // the average
int sensor;                   //The readings from the sensor

//Constants and variables for the Servo
Servo servo1;                 // Variable for the servo
int servoAngle = 0;           // Initial servo position 0 degrees
int servo;                    // The position of the servo where reading was made of closest object
int op_delay = 500;        //Delay between each operation

//These are the constants for selection of closest limits
int a = 402;      //value 400 set to maximum range of sensor in cm
int b = 401;
int c = 400;
int a1 = 0;        //Respective angles, initialised as zero
int b1 = 0;
int c1 = 0;

//For calculations of triangle sides
int temp = 1;          //Temporary for debug -- changed to value to read read off sensor values X times
int x, y;              //The constants of a, b or c for the Xside functions
int alpha;             //The constant that holds the value of the angle for Xside functions
double sideA, sideB, sideC;    //The returned values of the length of each side of triangle

//Determination of closest pole
int dist;
int distmax;

//Constants and variable decalaration for motor control
#define motor1Dir 7
#define motor2Dir 8
#define motor1PWM 9
#define motor2PWM 10
#define motor1Enable 11
#define motor2Enable 12
#include <avr/interrupt.h>    //Inclusion of relevant interrupt library


  int newAngle;      //This Angle value will be returned from the sensor mesurement function
  int newAngle2;
  int ref, align;

  int i = 0, facing = 0;             //This variable is for the servo angle array elements
  int xAngle;            //This is the angle that will be written to the servo whilst in motion
  int sensed;                  //New variable to store the expected sensed values
//Declaring preset values obtained from the sensor
double Xmax;
double Ymax;
double Xaxis, Yaxis;

//Initial program run once, when the device is powered on
void setup() 
{
  servo1.attach(5);                 //Pin 4 where the server is connected 
  initMotorDriver();                //Sets the motor pins up, ready to work.
  stopMotors();                     //Motors start with no motion
  // make the init pin an output:
  pinMode(TxPin, OUTPUT);
  // make the echo pin an input:
  pinMode(RxPin, INPUT);
  digitalWrite(TxPin, LOW);
  delayMicroseconds(50);
  Serial.begin(115200);
}

//The main function of the program
void loop() 
{  
    ScanArea();                //Calls for the function that does a 180 degree scan
    delay(1000);
    
    checkPoles();              //Mostly for debugging, runs through the determined poles found from previous function
    delay(1000);
    
    calculation_of_sides();            //Calls for the calculation function for sides of triangle from the data from previous step
    
    det_XYmax();                  //Calls for the function that provides X and Y maximum ranges.
    
    Pole_selection();             //Selects closest determined pole and sends DC motor control to that pole
    delay(1000);

//   faceForward();              //Make sure the vehicle is facing the right way
//   straight2pole();

        
    follow_path();        //Begins following pre-determined path
 
   while(1)
   {
     debugFunctions();
   }                    
 
}

/*********************************************************************************************
*      Callibration Routine                                                                  *
**********************************************************************************************/


/*#############################################################################################
# This Part of the code is where the different functions that are called in the loop function #
# will take place.          Sequentially presented                                            #
#############################################################################################*/

int Usound()                                  //Reading from ultrasonic sensor
{
      digitalWrite(TxPin, HIGH);                             // send signal
      delayMicroseconds(50);                                 // wait 50 microseconds for it to return
      digitalWrite(TxPin, LOW);                              // close signal
      pulseTime = pulseIn(RxPin, HIGH);                      // calculate time for signal to return
      distance = pulseTime/58;                               // convert to centimetres
    return distance;
}

//###############################################################End of Function##########################################

void ScanArea()                              //Begins a 180 degree area scan of whatever is in front of the device
{
  servo1.write(0);
  delay(50);
    for (servoAngle = 0; servoAngle <= 180; servoAngle++)
  {    
    servo1.write(servoAngle);
   for (index = 0; index <= Nreadings; index++)
   {
      sensor = Usound();                        //Runs the sensor formula to capture data from the ultrasonic transducer
      total = total + sensor;
      delay(10);
   }
   average = total/Nreadings;
    if (index >= Nreadings)  
    {
      index = 0;
      total = 0;
    }
    
    sensor_det_abc();                                        //Function to a
//*Debug Serial Output      Comment out when not required    
    Serial.print("X");                                       // print leading X to mark the following value as degrees
    Serial.print(servoAngle);                                // current servo position
    Serial.print("V");                                       // preceeding character to separate values
    Serial.println(average);                                  // average of sensor readings

   }
  
  for (servoAngle = 180; servoAngle >= 0; servoAngle--)
  {
    servo1.write(servoAngle);
   for (index = 0; index <= Nreadings; index++)
   {
      sensor = Usound();                        //Runs the sensor formula to capture data from the ultrasonic transducer
      total = total + sensor;
      delay(10);
   }
   average = total/Nreadings;
    if (index >= Nreadings)  
    {
      index = 0;
      total = 0;
    }
    sensor_det_abc();
//*Debug serial output     Comment out when not required   
    Serial.print("X");                                        // print leading X to mark the following value as degrees
    Serial.print(servoAngle);                                 // current servo position
    Serial.print("V");                                        // preceeding character to separate values
    Serial.println(average);                                  // average of sensor readings
//*/
  }
}

//###############################################################End of Function##########################################

void sensor_det_abc()                          //Determination of a, b and c from sensor readings == Function Needs Reviewing
{  
   if (average < a || average < b || average < c)    //Determination of the three limits
  {	    //Compares with the lowest value of limits
      if (average == a || average == b || average == c)
      {
      }
      else if (servoAngle < a1 + 40)
      {
        if (average < a)
        {
          a = average;
          a1 = servoAngle;
	}
      }
      else if (servoAngle < b1 + 40)
      {
	if (average < b)
        {
	  b = average;
	  b1 = servoAngle; 
        }
      }
      else if (servoAngle < c1 + 40)
      {
        if (average < c)
	{
	c = average;
	c1 = servoAngle;
	}
      }
      else if (a > b && a > c)	                 //checks if the value of a is the largest and replaces if so
      {
        a = average;                            //Saves the value of the sensor in a (smallest) i.e closest
        a1 = servoAngle;
       }
      else if (b > a && b > c)
      {
        b = average;
        b1 = servoAngle;			
        }
      else if (c > a && c > b)
      {
        c = average;
        c1 = servoAngle;
        }
    }

    // output For Debug Commented out when not required
  Serial.print(a, DEC);   Serial.print(" cm at ");    Serial.print(a1, DEC);      Serial.print(" degrees   |");
  Serial.print(b, DEC);   Serial.print(" cm at ");    Serial.print(b1, DEC);      Serial.print(" degrees   |");
  Serial.print(c, DEC);   Serial.print(" cm at ");    Serial.print(c1, DEC);      Serial.print(" degrees   |");
        Serial.println();
}

//###############################################################End of Function##########################################

void calculation_of_sides()                        //Function that calculates the values of sideA, sideB and sideC
{
   		x = a,
		y = b;
		alpha = abs(a1-b1);
		sideA = Xside (x, y, alpha);
		
		x = b;
		y = c;
		alpha = abs(b1-c1);
		sideB = Xside(x, y, alpha);  
		
		x = c;
		y = a;
		alpha = abs(c1-a1);
		sideC = Xside(x, y, alpha);

        //Debugging serial data for calculations1
        Serial.println();
    Serial.print(sideA, DEC);
    Serial.print(" cm is the distance between first and second value "); 
        Serial.println();
    Serial.print(sideB, DEC);
    Serial.print(" cm is the distance between second and third value ");
        Serial.println();
    Serial.print(sideC, DEC);
    Serial.print(" cm is the distance between third and first value ");
        Serial.println();
}

double Xside (double x, double y, double alpha)		  //Function for Calculations for the Unknown side
{

	double Xside = sqrt((x*x)+(y*y)-(2*x*y*cos(alpha*3.14159265/180)));    //Formula for determination of Xside
	return Xside;                                      //Returns the value of the unknown side required
}

//###############################################################End of Function##########################################

void det_XYmax()                    //This function determines the maximum values of X and Y from previous calculation
{
	//new comparisson of values			
	if (sideA>sideB && sideA>sideC)
	{
		Xmax = sideB;
		Ymax = sideC;
	}
	else if (sideB>sideA && sideB>sideC)
	{
		Xmax = sideA;
		Ymax = sideC;
	}
	else if(sideC>sideA && sideC>sideB)
	{
		Xmax = sideB;
		Ymax = sideA;
	}
}

//###############################################################End of Function##########################################

void Pole_selection()      //This functions chooses any pole at random preferably closest.
{
  if (a < b && a < c)        //Closest detected object
  {
    dist = a;
    servo1.write(a1);
    newAngle = a1;           //Set the new angle to be used for facing forward
    Serial.print("Pole in memory 'a' selected");   //For Debug serial reading of chosen pole
    faceForward();          //wheels 1st face forward if not already
    delay(op_delay);        //Delay between each operation
    straight2pole();        //Motor control function for forward motion till pole to be placed here
  }
  else if (b < a && b < c)    //Closest determined object
  {
    dist = b;
    servo1.write(b1);
    newAngle = b1;           //Set the new angle to be used for facing forward
    Serial.print("Pole in memory 'b' selected");   //For Debug serial reading of chosen pole
    faceForward();          //wheels 1st face forward if not already
    delay(op_delay);        //Delay between each operation
    straight2pole();            //Motor control function for forward motion till pole to be placed here
  }
  else if (c < a && c < b)     //Closest determined object
  {
    dist = c;
    servo1.write(c1);
    newAngle = c1;           //Set the new angle to be used for facing forward
    Serial.print("Pole in memory 'c' selected");   //For Debug serial reading of chosen pole
    faceForward();          //wheels 1st face forward if not already
    delay(op_delay);        //Delay between each operation
    straight2pole();        //Motor control function for forward motion till pole to be placed here
  }
}

/*===========================================================================================================
=            Predefined Servo Array determined in an Excel spreadsheet                                      =
=        This Array determines what angles should be sent to the servo                                      =
============================================================================================================*/

    int servoAngle_array[145] = {48,13,7,2,1,3,5,19,31,48,48,39,28,8,5,7,11,34,
                              43,48,48,45,38,14,9,11,17,41,46,48,48,47,42,20,
                              13,15,22,44,47,48,48,47,45,25,16,18,27,45,47,48,
                              48,48,46,29,20,21,30,46,48,48,48,48,46,32,23,24,
                              33,47,48,48,48,48,47,34,26,27,36,47,48,48,48,48,
                              47,37,28,29,38,47,48,48,48,48,47,38,30,31,39,47,
                              48,48,48,48,48,40,32,33,40,48,48,48,48,48,48,41,
                              34,34,41,48,48,48,48,48,48,42,35,36,42,48,48,48,
                              48,48,48,43,36,37,43,48,48,48,48,48,48,43,38
                              };
  //Above array contains 145 elements, which according to the value of i for the servo position during motion on the track
  
  int expected_sensor_dist[145] = {10,30,60,240,390,390,240,60,40,20,40,50,70,240,390,390,
                                240,80,60,50,60,70,90,250,400,400,250,100,80,80,90,100,
                                110,260,400,400,260,120,110,110,120,120,130,270,410,410,
                                270,150,140,130,150,150,160,280,420,420,290,170,160,160,
                                180,180,190,300,430,430,310,200,190,190,200,210,210,310,
                                440,450,320,230,220,220,230,230,240,330,450,460,340,250,
                                250,250,260,260,270,350,470,480,360,280,270,270,290,290,
                                290,370,480,490,380,310,300,300,320,320,320,400,500,510,
                                410,330,330,330,340,340,350,420,520,530,430,360,360,360,
                                370,370,380,440,540,550,450,390,390,390,400,400,400,470,
                                560};
      //Expected sensor distance array same as for the servo angle array. works backwards every 5 nos.

//###############################################################End of Function##########################################

//Function that determines where the array should be started from, according to the maximum distance of Y
void initial_location()
{
  if (Ymax <= 400 && Ymax >= 370)      //Considering the maximum distance between those values, begin array on 1
  {
    i = 4;
  }
  else if (Ymax < 370 && Ymax >= 340)
  {
    i = 3;
  }
  else if (Ymax < 340 && Ymax >= 160)
  {
    i = 2;
  }
  else if (Ymax < 160 && Ymax >= 40)
  {
    i = 1;
  }
  else if (Ymax < 40 && Ymax >= 0)
  {
    i = 0;
  }
}

/************************************************************************************************************
*               THis is the Motor Handling code (Still being debugged)                                      *
*                        DC MOTOR CONTROL PARAMETERS                                                        *
*************************************************************************************************************/

//Function that initiates the motors
void initMotorDriver()
{
  pinMode(motor1Dir, OUTPUT);
  pinMode(motor2Dir, OUTPUT);

  pinMode(motor1Enable, OUTPUT);
  pinMode(motor2Enable, OUTPUT);
  digitalWrite(motor1Enable,HIGH);
  digitalWrite(motor2Enable,HIGH);
}

//Function that determines the motor speeds
void setMotorVel(int directn, int PWMcontrol, int velocity)
{
  if (velocity >= 255) velocity = 255;
  if (velocity <= -255) velocity = -255;

  if (velocity == 0)
  {
    digitalWrite(directn, HIGH);
    digitalWrite(PWMcontrol, HIGH);
  }
  else if(velocity <0){  // Reverse
    digitalWrite(directn, HIGH);
    analogWrite(PWMcontrol, velocity);
  }
  else if(velocity >0){ // Forward
    digitalWrite(directn,LOW);
    analogWrite(PWMcontrol, velocity);      //Function to be debugged and monitored, may affect speeds! Motor tests required.
  }
}

//Function that controls the directions of the motors
void setLeftMotorSpeed(int velocity)
{
  setMotorVel(motor1Dir, motor1PWM, velocity);

}

void setRightMotorSpeed(int velocity){
  setMotorVel(motor2Dir, motor2PWM, velocity);
}

//Direction controllers
void goForward()
{
  setLeftMotorSpeed(255);
  setRightMotorSpeed(255);
     Serial.print("Vehicle going forward");   //For debug
     Serial.println();
}

void goBackward()
{
  setLeftMotorSpeed(-255);
  setRightMotorSpeed(-255);
     Serial.print("Vehicle going backrward");   //For debug
     Serial.println();
}

void goRight()
{
  setLeftMotorSpeed(255);
  setRightMotorSpeed(0);
    Serial.print("Turning Right");   //For debug
    Serial.println(); 
}

void SgoRight()        //Go right in the same location
{
  setLeftMotorSpeed(255);
  setRightMotorSpeed(-255);
}

void goLeft()
{
  setLeftMotorSpeed(0);
  setRightMotorSpeed(255);
    Serial.print("Turning left");   //For debug
    Serial.println();  
}

void SgoLeft()        //Go left in the same location
{
  setLeftMotorSpeed(-255);
  setRightMotorSpeed(255);
}

void stopMotors()
{
   setLeftMotorSpeed(0);
   setRightMotorSpeed(0);
     Serial.print("Stopping motors");   //For debug
     Serial.println();
}

//###########################################End of Function##########################################

//Function for predefined motion automation

void follow_path()
{
int siz = 0, n=0, i0 = 0, alt_angle = 0, turns = 0, starting = 0, limitVal;
initial_location();                        //Checks for the initial position from maximum distance
//  i = 1;           //First time force for debug
//calculations to determine the number of turns before ending process.
//Xmax is known from det_XYmax function
  limitVal =(Xmax * 28)/400;  //28 was determined as max no. of turns in 400cm
sensor = Usound();                        //Check the distance

      checkPoleMotion();          //Confirm that it is in a straight line if yes, continue
      servo1.write(180);            //Position servo to first pole
       Serial.print("setting new angle of servo at:");         //for debug 
       Serial.print(alt_angle, DEC); 
       Serial.println();  
         do	            //Motion made towards maximum Y limit doing relevant functions
        {
        sensor = Usound();                        //Check the distance 
        Serial.print(sensor, DEC);
        Serial.println();
         }  
         while (sensor > (15 + siz));
       delay(500);                 //Wait for the vehicle to get right at pole
       stopMotors();               //First stops the motors
  do
    {
      if (starting == 1)          //First round don't run
      {
      checkPoleMotion();          //Confirm that it is in a straight line if yes, continue
      
       servo1.write(alt_angle);            //Position servo to first pole
       Serial.print("setting new angle of servo at:");         //for debug 
       Serial.print(alt_angle, DEC);       
       Serial.println();
       
         do	            //Motion made towards maximum Y limit doing relevant functions
        {
        sensor = Usound();                        //Check the distance 
        Serial.print(sensor, DEC);
        Serial.println();
         }  
         while (sensor > (15 + siz));
       delay(500);                 //Wait for the vehicle to get right at pole
       stopMotors();               //First stops the motors
      
         //switsching angles   and updating values for next time round the loop   
        if(alt_angle == 0)
        {
          alt_angle = 180;
        }
        else
        {
          alt_angle = 0;
        }        
      }
       
//      goForward();                //Go Forward
//      delay(500);                //wait 1s for the vehicle to be out of sight of pole

         starting = 1;                //Such that closed loop is applicable next loops
 //The vehicle has reached the end of a forward track
       delay(1000);                //wait for the servo to get there
       servo1.write(alt_angle);            //Position servo to first pole
       Serial.print("setting new angle of servo at:");         //for debug 
       Serial.print(alt_angle, DEC);       
       Serial.println();
       delay(4000);                //wait for the servo to get there    
  
      if (turns == 0)
      {
       Serial.print("turns = 0 loop");    //for debug        //Runs the function that does the closed loop motion
       Serial.println();
       goLeft();                //Turning left at pole
       siz = siz+10;
       Serial.print(siz, DEC);
       Serial.println();
       turns = 1;              //Sets the value of turns so that the next time around it turns Left
       delay(500);              //Ignore the first couple of readings
           do	//Motion made towards maximum Y limit doing relevant functions
          {
            sensor = Usound();                        //Check the distance
            Serial.print(sensor, DEC);
            Serial.println();
           }  
           while (sensor > (15 + siz));
        }
        else
        {
         Serial.print("turns = 1 loop");    //for debug        //Runs the function that does the closed loop motion
         Serial.println();
         goRight();                //Turning right at pole
         siz = siz+10;
         Serial.print(siz, DEC);
         Serial.println();
          turns = 0;                 //Sets the value of turns so that the next time around it turns right
          delay(500);              //Ignore the first couple of readings          
           do	//Motion made towards maximum Y limit doing relevant functions
          {
            sensor = Usound();                        //Check the distance
            Serial.print(sensor, DEC);
            Serial.println();
           }  
           while (sensor > (15 + siz));
          }
  //The vehicle is now adjacent in the next track to repeat motion
        stopMotors();               //First stops the motors

         Serial.print("i0 =");    Serial.print(i0, DEC);     Serial.println();  //for debug    
         i0++;
        delay(8000);                //wait for the servo to get there    
    }
    while (i0 < limitVal);      //Repeats the process until close to the limit


//Job Complete, motors being stopped
Serial.print("Stopping motors -- Job complete Boss :-)");   //For debug
Serial.println();
        stopMotors();
}

//This function manages the closed loop motion during the track where the servo faces the pole and checks reading for match
void checkPoleMotion()
{
int ii0, k = 0;
ii0 = i+1;      //Stores the first value of i
   
   if (facing == 0)    //If going forward, add 90 degrees
   {  
     do	//Motion made towards maximum Y limit doing relevant functions
     {
       xAngle = servoAngle_array[i] + 90;      //xAngle is the angle that will be written to the servo 4-i because data to be accessed in opp direction

           Serial.print("Servo @ angle:");    //for debug
           Serial.print(xAngle, DEC);            //for debug
           Serial.println();
        servo1.write(xAngle);                   //Sets the servo in a new position
        delay(1000);                             //Wait for the servo to get to the new angle
        sensed = expected_sensor_dist[i];    //Start the maximum distance expected as Ymax and give extra 1 or 2 cm due to possible error
 
       goForward();                              //Can be placed on a condition in order for it to occur only once
        do	//Motion made towards maximum Y limit doing relevant functions
        {
          sensor = Usound();                        //Check the distance
          Serial.print(sensor, DEC);
          Serial.println();
        }  
        while (sensor > sensed+5);
        i--;                                    //Jumps to the previous array element
    }
    while(i >= k);                    //The number of times the servo has to change angles before it reaches the pole
    k = i+6;
    if (ii0-1 == 4)
    {
	i = ((ii0-1) +i+2);
    }
    else if (ii0-1 == 3)
    {
	i = ((ii0-1) +i+4);
    }
    else if (ii0-1 == 2)
    {
        i = ((ii0-1) +i+6);
     }
    else if (ii0-1 == 1)
    {
      	i = ((ii0-1) +i+8);
    }
    else if (ii0-1 == 0)
    {
	i = ((ii0-1) +i+10);
    }
    facing = 1;
   }
     else                //Going opposite way, then subtract from 90 degrees
    {
      do
      {
       xAngle = 90 - servoAngle_array[i];      //xAngle is the angle that will be written to the servo 4-i because data to be accessed in opp direction
         
         Serial.print("Servo @ angle:");    //for debug
         Serial.print(xAngle, DEC);            //for debug
         Serial.println();
        servo1.write(xAngle);                   //Sets the servo in a new position
        delay(1000);                             //Wait for the servo to get to the new angle
        sensed = expected_sensor_dist[i];    //!!Start the maximum distance expected as Ymax and give extra 1 or 2 cm due to possible error
      
       goForward();                              //Can be placed on a condition in order for it to occur only once
        do	//Motion made towards maximum Y limit doing relevant functions
        {
          sensor = Usound();                        //Check the distance
          Serial.print(sensor, DEC);
          Serial.println();
        }  
        while (sensor > sensed+5);
        i++;                                    //Jumps to the next array element
    }
    while(i <= k+4);                    //The number of times the servo has to change angles before it reaches the pole
    k = i;
    i = ((ii0-1)+i);
    facing = 0;
    }

}

//###############################################################End of Function##########################################

//Funcction for the wheel motors to face the same direction as that of the sensor (to be debugged in actual hardware)

void faceForward()
{  
	ref = 90;			//servo @0 degrees initial position
        servo1.write(ref);
//The entire function would be in a closed loop with Usonic transducer data
//i.e when the Usonic transducer is at ref, no degree (or few degrees tolerance) away, then the wheels are aligned if not, this continues
        stopMotors();
    if (newAngle != ref)
    {
	if (newAngle < ref)				
	{
	 do		//Calculations may be required depending on DC Motors to determine how long or fast the motors need to go for the new position
	  {
		SgoRight();
//                delay(50);
                sensor = Usound();
                Serial.print("Turning Right Loop value:");
                Serial.print(newAngle, DEC);
                Serial.println();
		//This part of the code in Arduino ir replace by the motor control function for both wheels turning opposite side +255L && -255R
	  }
          while(dist-5 < sensor && sensor < dist+5);
		stopMotors();
                Serial.print("Stopping motors");   //For debug
                Serial.println();
		//Enter Arduino motor control to stop the motors as the new direction has be accomplished
          }
        else if (newAngle > ref)
        {
       	 do		//Calculations may be required depending on DC Motors to determine how long or fast the motors need to go for the new position
	  {
		SgoLeft();
                sensor = Usound();
//                delay (50);
                Serial.print("Turning Left Loop value:");
                Serial.print(newAngle, DEC);
                Serial.println();
		//This part of the code in Arduino ir replace by the motor control function for both wheels turning opposite side +255L && -255R
	  }
         while (dist-5 < sensor && sensor < dist+5);
		stopMotors();
                delay (10);
                Serial.print("Stopping motors");   //For debug
                Serial.println();
		//Enter Arduino motor control to stop the motors as the new direction has be accomplished
          }
    }
  
}

//###############################################################End of Function##########################################

void straight2pole()          //Function doesn't work!! debug me!
{ 
    CL_motion(1, 10);    /*goForward stop when sensor value is 10*/;      
                         //In the function for closed loop motion, represents go forward
}

// Closed loop motion function 
void CL_motion(int CLmotion, int wen2stop)
{
  int expt_dist;              //This is the closed loop distance variable
  expt_dist = Usound();       //Saves the range distance from reading into limiting variable
  do
  {
  	//Arduino Motor code for forward motion
    if (CLmotion = 1)        //Motors drive forward motion
    {
     sensor = Usound();         //Check the sensor value
       if (sensor <= expt_dist)   //If the distance meets the condition
       {
         expt_dist = sensor;        //make the velue of the new reading the expected distance 
         goForward();            //Make the desired closed loop motion goForward
          Serial.print("CLmotion = 1");   //For debug
          Serial.println();
        }
    else
    {
    //This section of the code considers the device having lost the pole and tries to find it again
      stopMotors();
     //To be completed
      }
          delay (100);                  //Wait for motor to get to point required (has to be determined
    distmax = Usound();                //Closing the loop --> Checking sensor data as it goes
        Serial.print("Distance max:");
        Serial.print(distmax, DEC);
        Serial.println();
    }
    else if (CLmotion = 2)    //Motors drive right 180 degrees motion
    {
      Serial.print("CLmotion = 2");   //For debug
      Serial.println();
      goRight();              //Turning right at the first pole
      if (sensed-5 < sensor && sensor < sensed+5)         //Checks if a pole was determined within the specified distance
        {
          distmax = 8;                                    //Sets the value of distmax that ends the loop
        }
      else
      {
        delay (100);                  //Wait for motor to get to point required (has to be determined
        distmax = Usound();                //Closing the loop --> Checking sensor data as it goes
        Serial.print("Distance max:");
        Serial.print(distmax, DEC);
        Serial.println();
      }
    }
    else if (CLmotion = 3)    //Motors drive left 180 degrees motion
    {
      Serial.print("CLmotion = 3");   //For debug
      Serial.println();
      goLeft();                //Turning left at the first pole
      if (sensed-5 < sensor && sensor < sensed+5)         //Checks if a pole was determined within the specified distance
        {
          distmax = 8;                                    //Sets the value of distmax that ends the loop
        }
      else
      {
        delay (100);                  //Wait for motor to get to point required (has to be determined
        distmax = Usound();                //Closing the loop --> Checking sensor data as it goes
        Serial.print("Distance max:");
        Serial.print(distmax, DEC);
        Serial.println();
      }
    }
   }
   while (distmax > wen2stop);                    //The vehicle goes forward until it's too close to the pole
    	//The vehicle is close to turning point one direction
        Serial.print("Stopping motors");   //For debug
        Serial.println();
      stopMotors();  
}

/**********************************************************************************************************************
*                  Debugging Functions - - - - - - > For servicing only           Uncomment when not required/in use  *
***********************************************************************************************************************/

void debugMotors()
{
  char l;
  int adcVal;

  l = Serial.read();
  
  switch(l){
    case 'f' :
      goForward();
      break;
    case 'r':
      setLeftMotorSpeed(-255);
      setRightMotorSpeed(-255);
      break;
    case 's':
      setLeftMotorSpeed(0);
      setRightMotorSpeed(0);
      break;
    case 't':
      goRight();
    break;
    case 'l':
      goLeft();
    break;
    case 'w':
      SgoRight();
    break;
    case 'q':
      SgoLeft();
    break;
    case 'z':
      debugFunctions();
    break;
  }
}


//###############################################################End of Function##########################################

void checkPoles()
{
  //Debug function mainly, for the servo to run through A B and C
  Serial.println();
  servo1.write(a1);
  sensor = Usound();
  Serial.print(sensor);
  delay (1000);

  Serial.println();  
  servo1.write(b1);
  sensor = Usound();
  Serial.print(sensor);
  delay (1000);

  Serial.println();  
  servo1.write(c1);
  sensor = Usound();
  Serial.print(sensor);
  delay (1000);
  
  servo1.write(0);
}

//###############################################################End of Function##########################################

void debugFunctions()
{
  char c;
  c = Serial.read();
    
  switch(c)
  {
    case 'a' :
      ScanArea();
    break;
    case 'b':
      checkPoles();
    break;
    case 'c':
      calculation_of_sides(); 
    break;
    case 'd':
      det_XYmax();
    break;
    case 'e':
      Pole_selection();  
    break;
    case 'f':
      faceForward();  
    break;
    case 'g':
      straight2pole();
    break;
    case 'h':
      follow_path();
    break;
    case 'A':
      Serial.print("Welcome to the motor debugger");   //For debug
      debugMotors();
    break;    
  }
}
