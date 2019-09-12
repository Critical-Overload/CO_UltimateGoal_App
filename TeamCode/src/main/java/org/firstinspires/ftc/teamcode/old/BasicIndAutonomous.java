package org.firstinspires.ftc.teamcode.old;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "BasicIndAutonomous")

/*
This is a basic autonomous program.
The robot moves forward from the "far" landing zone
And then moves sideways into the crater
*/

public class BasicIndAutonomous extends LinearOpMode{

    //init motors
    private DcMotor motorFrontRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackLeft;
    private DcMotor motorBackRight;
    private DcMotor liftMotor;
    
    @Override
    
    public void runOpMode() throws InterruptedException {
        
        //define motors
        motorFrontRight = hardwareMap.dcMotor.get("FR");
        motorFrontLeft = hardwareMap.dcMotor.get("FL");
        motorBackLeft = hardwareMap.dcMotor.get("BL");
        motorBackRight = hardwareMap.dcMotor.get("BR");
        liftMotor = hardwareMap.dcMotor.get("lift");
        
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        
        waitForStart();
        
        moveLift(-.5, 3.0);
        moveTime(.5, 1.0);
        moveSideTime(-.5, 3.5);
    }
    
    public void moveTime(double power, double seconds){
        //Parameters: power to run at, seconds to run
        motorFrontRight.setPower(power);
        motorFrontLeft.setPower(power);
        motorBackRight.setPower(power);
        motorBackLeft.setPower(power);
        
        //uses seconds param to calculate milliseconds
        long y = (int) Math.rint(seconds * 1000);
        
        //waits for y milliseconds
        sleep(y);
        
        //stops all motors
        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackRight.setPower(0);
        motorBackLeft.setPower(0);
        
        
    }
    public void moveSideTime(double power, double seconds){
        //Parameters: power (positive for left, negative for right), seconds
        motorFrontRight.setPower(power);
        motorFrontLeft.setPower(-power);
        motorBackRight.setPower(-power);
        motorBackLeft.setPower(power);
        
        //uses seconds param to calculate milliseconds
        long y = (int) Math.rint(seconds * 1000);
        
        //waits for y milliseconds
        sleep(y);
        
        //stops all motors
        motorFrontRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackRight.setPower(0);
        motorBackLeft.setPower(0);
    }
    
    public void moveLift (double power, double seconds){
        
        //Parameters: power and seconds
        liftMotor.setPower(power);
        
        //uses seconds param to calculate milliseconds
        long y = (int) Math.rint(seconds * 1000);
        
        //wait for y milliseconds
        sleep(y);
        
        //stops lift motor
        liftMotor.setPower(0);
    }
    

}