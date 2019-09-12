//defines functions for autonomous
package org.firstinspires.ftc.teamcode.old;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


public class FunctionsForAutonomous{

    private DcMotor motorFrontRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackRight;
    private DcMotor motorBackLeft;
    private DcMotor liftMotor;
    private Servo bucketServo;
    private DigitalChannel touchSensor;
    
    public FunctionsForAutonomous (DcMotor motorFrontRight, DcMotor motorFrontLeft, 
    DcMotor motorBackRight, DcMotor motorBackLeft, DcMotor liftMotor, Servo 
    bucketServo, DigitalChannel touchSensor){
        this.motorFrontRight = motorFrontRight;
        this.motorFrontLeft = motorFrontLeft;
        this.motorBackRight = motorBackRight;
        this.motorBackLeft = motorBackLeft;
        this.liftMotor = liftMotor;
        this.bucketServo = bucketServo;
        this.touchSensor = touchSensor;
        
        this.motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    
    public double hexTicks = 288; //for Hex motor
    public double torqueTicks = 1440; //for Torquenado motor
    
    public void reverseMotor(DcMotor motor){
        motor.setDirection(DcMotor.Direction.REVERSE);
    }
    
    public void move(double inches) throws InterruptedException{
        //Parameters: inches
        double inPerCy = 12; //inches per cycle
        int ticks = (int) ((inches/inPerCy)*288); //converts inches to ticks
        double frTarget = motorFrontRight.getCurrentPosition()+ticks; //remember fr target for while loop
        //add amount of movement needed to each motor's target position
        motorFrontRight.setTargetPosition(motorFrontRight.getCurrentPosition()+ticks);
        motorFrontLeft.setTargetPosition(motorFrontLeft.getCurrentPosition()+ticks);
        motorBackRight.setTargetPosition(motorBackRight.getCurrentPosition()+ticks);
        motorBackLeft.setTargetPosition(motorBackLeft.getCurrentPosition()+ticks);
        while(motorFrontRight.getCurrentPosition() < (frTarget-10)){
            Thread.sleep(10); //waits for example motor to be within 10 tick threshold
        }
    }
    public void moveSide(double inches) throws InterruptedException{
        //Parameters: inches (positive for left, negative for right)
        double inPerCy = 11; //inches per cycle
        int ticks = (int) ((inches/inPerCy)*288); //converts inches to ticks
        double frTarget = motorFrontRight.getCurrentPosition()+ticks; //remember fr target for while loop
        //add amount of movement needed to each motor's target position
        motorFrontRight.setTargetPosition(motorFrontRight.getCurrentPosition()+ticks);
        motorFrontLeft.setTargetPosition(motorFrontLeft.getCurrentPosition()-ticks);
        motorBackRight.setTargetPosition(motorBackRight.getCurrentPosition()-ticks);
        motorBackLeft.setTargetPosition(motorBackLeft.getCurrentPosition()+ticks);
        while(motorFrontRight.getCurrentPosition() < (frTarget-10)){
            Thread.sleep(10);//waits for example motor to be within 10 tick threshold
        }
    }
    public void turn(double degrees) throws InterruptedException{
        //Parameters: degrees (positive for clockwise,
        // negative for counterclockwise)
        double degPerCy = 45; //degrees per cycle
        int ticks = (int) ((degrees/degPerCy)*288); //converts degrees to ticks
        double frTarget = motorFrontRight.getCurrentPosition()-ticks; //remember fr target for while loop
        //add amount of movement needed to each motor's target position
        motorFrontRight.setTargetPosition(motorFrontRight.getCurrentPosition()-ticks);
        motorFrontLeft.setTargetPosition(motorFrontLeft.getCurrentPosition()+ticks);
        motorBackRight.setTargetPosition(motorBackRight.getCurrentPosition()-ticks);
        motorBackLeft.setTargetPosition(motorBackLeft.getCurrentPosition()+ticks);
        while(motorFrontRight.getCurrentPosition() > (frTarget+10)){
            Thread.sleep(10);//waits for example motor to be within 10 tick threshold
        }
    }
    /*public void land() throws InterruptedException{
        //lands the robot
        liftMotor.setPower(-0.5);
        
        Thread.sleep(3000);
        
        completeStop();
    }
    //public void armIntakePosition(){
        
    //}
    public void resetBucket(){
        //resets the bucket
        //moves bucket back until it hits touch sensor
        while(touchSensor.getState() == true){
            bucketServo.setPower(-0.1);
        }
    }*/
    public void completeStop(){
        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackRight.setPower(0);
        motorBackLeft.setPower(0);
        liftMotor.setPower(0);
        //completely stops all motors
    }
    public void fullPower(){
        motorFrontRight.setPower(1);
        motorFrontLeft.setPower(1);
        motorBackRight.setPower(1);
        motorBackLeft.setPower(1);
        liftMotor.setPower(1);
        //runs all motors max power
    }
    public void halfPower(){
        motorFrontRight.setPower(0.5);
        motorFrontLeft.setPower(0.5);
        motorBackRight.setPower(0.5);
        motorBackLeft.setPower(0.5);
        liftMotor.setPower(0.5);
        //runs all motors half power
    }
    public void ctrlPower(){
        motorFrontRight.setPower(0.3);
        motorFrontLeft.setPower(0.3);
        motorBackRight.setPower(0.3);
        motorBackLeft.setPower(0.3);
        liftMotor.setPower(0.3);
        //runs all motors at controlled power
    }    
    //public void scanMinerals(){
        //left, right, center
    //}
    
}
