FEHMotor right_motor(FEHMotor::Motor0,9.0);
FEHMotor left_motor(FEHMotor::Motor1,9.0);

FEHServo buttonServo(FEHServo::Servo0); //declaring the servo to port 0.
FEHServo leverServo(FEHServo::Servo7);//declaring the servo to 7.
AnalogInputPin cds(FEHIO::P2_0); //declaring cds cell

void changeMiniServo();
void move_forward(float percent, float counts);
void turn_right(float percent, float counts);
void turn_left(float percent, float counts);
void stopMotors();

//start main ***************************************************************************\
int main()
{
    // get the voltage level and display it to the screen
    LCD.WriteLine( Battery.Voltage() );
    Sleep( 0.5 );

    //Code to make robot read the start light to move
    while(!(cds.Value()>0.300 && cds.Value()<0.358));
    buttonServo.SetMin(590);
    buttonServo.SetMax(2330);
    buttonServo.SetDegree(0);

    //Moving Robot to jukebox
    int motor_percent = 25; //intializes motor percent
    float counts = 40.49; //intializes encoder counts

    move_forward(motor_percent, counts*18.5);
    //Approaching the light.
    /*while(!(cds.Value()>0 && cds.Value()<0.23) || !(cds.Value()>1.35 && cds.Value()<1.39)){
        //code to make robot read the light, approach the to press jukebox button
        move_forward(-motor_percent,counts*0.1); //robot moves backward around 4 inches
    }*/

    stopMotors();
    Sleep(2.0);
    //Changing the orientation of buttonServo to read light
    changeMiniServo();

    left_motor.SetPercent(25);
    Sleep(1.80);

    //Moving robot closer to jukebox
    move_forward(-motor_percent,counts*10.5); //robot moves backward for 2 inches

    //moving robot to and up the Ramp
    move_forward(motor_percent,counts*4); //robot moves backwards for about 2 inches
    turn_right(motor_percent,counts*5); //makes a 90 degree right turn that's about 6 inches
    move_forward(motor_percent,counts*8.75); //robot moves forward around 8 inches
    turn_right(motor_percent,counts*5.5);//makes a 90 degree left turn that's about 6
    move_forward(-motor_percent*2,counts*35); //robot moves forward around 30 inches up the ramp

    //Move the robot down the ramp
    Sleep(0.5);
    move_forward(motor_percent,counts*30);

}
//end main*****************************************************************************

void changeMiniServo(){
    LCD.Clear();
    LCD.WriteLine("CDS Value: " );
    LCD.Write(cds.Value());
    float x = cds.Value();
    if(x>0.00 && x<0.8) { //if cdS cell picks up red
       LCD.WriteLine("Red Light");
      buttonServo.SetDegree(0);

    }else if(x>0.8 && x<2.24){
      buttonServo.SetDegree(180);
      LCD.WriteLine("Blue Light");
    }
}

void move_forward(float percent, float counts) //using encoders
{
    //Reset encoder counts
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();

   //Set both motors to desired percent
    right_motor.SetPercent(-percent); //right motor is oriented backwards, so negative means forward
    left_motor.SetPercent(percent);

   //While the average of the left and right encoder is less than counts,
    //keep running motors
    while((left_encoder.Counts() + right_encoder.Counts()) / 2. < counts);

   //Turn off motors
    right_motor.Stop();
    left_motor.Stop();
}

void turn_right(float percent, float counts) {
    //Reset encoder counts
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();

   //Set both motors to desired percent
    right_motor.SetPercent(percent);
    left_motor.SetPercent(percent);

    while((left_encoder.Counts() + right_encoder.Counts()) / 2. < counts);

    //Turn off motors
    right_motor.Stop();
    left_motor.Stop();

}

//This is the function used to turn the robot counterclockwise.
void turn_left(float percent, float counts) {
    //Reset encoder counts
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();

   //Set both motors to desired percent
    right_motor.SetPercent(-percent);
    left_motor.SetPercent(-percent);

    while((left_encoder.Counts() + right_encoder.Counts()) / 2. < counts);

    //Turn off motors
    right_motor.Stop();
    left_motor.Stop();
}

//this function is to stop the motors
void stopMotors(){
    left_motor.Stop();
    right_motor.Stop();

}
