//Include Proteus Libraries
#include <FEHIO.h>
#include <FEHMotor.h>
#include <FEHServo.h>
#include <FEHLCD.h>
#include <FEHRPS.h>
#include <FEHBattery.h>
#include <FEHAccel.h>
#include <FEHRandom.h>
#include <FEHBuzzer.h>
#include <FEHSD.h>
#include <FEHUtility.h>

//Define motor and encoder constants
#define CPR 318      //Counts per revolution
#define CPI 40.49    //Estimated counts per inch??
#define CPD 0.883      //Estimated counts per degree of rotation
#define P 1.0        //Power ratio
#define R 0.99       //Power reduction ratio of stronger wheel to allow weak wheel to catch up.

//Define direction multipliers
#define FORWARD 1    //Direction multiplier for forward drive
#define BACKWARD -1  //Direction multiplier for reverse drive
#define RIGHT 1      //Direction multiplier for right turns
#define LEFT -1      //Direction multiplier for left turns
#define ON 1         //Bump switch value??
#define OFF 0        //Bump switch value??

//Define cds thresholds
#define LR 0.00       //Lower red cds threshold??
#define UR 0.80       //Upper red cds threshold??
#define LB 1.0       //Lower blue cds threshold
#define UB 1.0       //Upper blue cds threshold      //Other cds thresholds??

//Define servo max and mins
#define BUTTON_MIN 590 //Button_servo min value??
#define BUTTON_MAX 2330 //Button_servo max value??
#define LEVER_MIN 900  //Button_servo min value??
#define LEVER_MAX 1600  //Button_servo max value??

//Declaring name encoders
DigitalEncoder right_encoder(FEHIO::P0_1);
DigitalEncoder left_encoder(FEHIO::P0_0);
//Declaring motors
FEHMotor right_motor(FEHMotor::Motor0,9.0);
FEHMotor left_motor(FEHMotor::Motor1,9.0);
//declaring servos
FEHServo buttonServo(FEHServo::Servo0); //declaring the servo to port 0.
FEHServo leverServo(FEHServo::Servo7);//declaring the servo to 7.
//declaring cds cell
AnalogInputPin cds(FEHIO::P2_0);
//Declaring bump switches
DigitalInputPin left_switch(FEHIO::P2_0);
DigitalInputPin right_switch(FEHIO::P2_1);

//Declaring prototypes
void changeButtonServo();
void reset_encoders();
void stopMotors();
void drive(int direction, float power, float inches);
void turn(int direction, float power, float degrees);



//start main ***************************************************************************
int main()
{

    //Clear the LCD screen
    LCD.Clear();
    // get the voltage level and display it to the screen
    LCD.WriteLine(Battery.Voltage());
    Sleep( 0.5 );

    //Code to make robot read the start light to move + calibrating servo motors
    while(!(cds.Value()>LR && cds.Value()<UR));
    buttonServo.SetMin(BUTTON_MIN);
    buttonServo.SetMax(BUTTON_MAX);
    buttonServo.SetDegree(0);

    leverServo.SetMin(LEVER_MIN);
    leverServo.SetMax(LEVER_MAX);
    leverServo.SetDegree(25); //gotta test to see how the plate will lift up

    //Code to make the robot up to the sink
    drive(FORWARD,25,10);
    turn(RIGHT,25,35);
    drive(FORWARD,50,30);


    //Code to make robot approach sink and dump the tray
    turn(LEFT,25,90);

    while(!right_switch.Value() || !left_switch.Value()){
        drive(FORWARD,25,20);
    }

        drive(BACKWARD,25,1);
        turn(LEFT,25,90);

     while(!right_switch.Value() || !left_switch.Value()){
        drive(FORWARD,25,8);
        }

     drive(BACKWARD,25,1);
     leverServo.SetDegree(45);
     Sleep(2.0);

     //Code to move robot to the hot plate
     leverServo.SetDegree(0);
     drive(BACKWARD,25,10);
     turn(LEFT,25,90);
     drive(FORWARD,25,24);
     turn(LEFT,25,90);
     leverServo.SetDegree(75);
     drive(FORWARD,25,20);





}
//end main*****************************************************************************

//***********************************Function definitions****************************************

void changeButtonServo(){
    LCD.Clear();
    LCD.WriteLine("CDS Value: " );
    LCD.Write(cds.Value());
    float x = cds.Value();
    if(x>LR && x<UR) { //if cdS cell picks up red
       LCD.WriteLine("Red Light");
      buttonServo.SetDegree(0);

    }else if(x>LB && x<UB){
      buttonServo.SetDegree(180);
      LCD.WriteLine("Blue Light");
    }
}

//Function to reset all encoders
void reset_encoders()
{
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();
}

//this function is to stop the motors
void stopMotors(){
    left_motor.Stop();
    right_motor.Stop();

}

//Make the robot move forward or backward with a specific amount of power and distance
void drive(int direction, float power, float inches) //using encoders
{
    //Reset encoder counts
    reset_encoders();
    float counts = CPI*inches;
   //Set both motors to desired percent
    right_motor.SetPercent(-R*direction*power); //right motor is oriented backwards, so negative means forward
    left_motor.SetPercent(P*direction*power);

   //While the average of the left and right encoder is less than counts,
    //keep running motors
    while((left_encoder.Counts() + right_encoder.Counts()) / 2. < counts);

   //Turn off motors
    stopMotors();
}

//Make the robot turn right or left, with specific amount of power and angle of turns
void turn(int direction, float power, float degrees) {

    //Declaring counts
    float counts = CPD*degrees;

    //Reset encoder counts
    reset_encoders();

   //Set both motors to desired percent
    right_motor.SetPercent(R*direction*power);
    left_motor.SetPercent(P*direction*power);

    while((left_encoder.Counts() + right_encoder.Counts()) / 2. < counts);

    //Turn off motors
    right_motor.Stop();
    left_motor.Stop();

}
