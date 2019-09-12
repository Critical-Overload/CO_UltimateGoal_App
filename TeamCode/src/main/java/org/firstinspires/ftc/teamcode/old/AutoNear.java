package org.firstinspires.ftc.teamcode.old;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "AutoNear")

public class AutoNear extends LinearOpMode{

    //declares motors and sensors
    private DcMotor motorFrontRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackLeft;
    private DcMotor motorBackRight;
    private DcMotor liftMotor;
    private Servo bucketServo;
    private DigitalChannel touchSensor;
    
    public void runOpMode() throws InterruptedException{
        
        //initializes motors and sensors
        motorFrontRight = hardwareMap.dcMotor.get("FR");
        motorFrontLeft = hardwareMap.dcMotor.get("FL");
        motorBackRight = hardwareMap.dcMotor.get("BR");
        motorBackLeft = hardwareMap.dcMotor.get("BL");
        liftMotor = hardwareMap.dcMotor.get("lift");
        bucketServo = hardwareMap.servo.get("bucketServo");
        touchSensor = hardwareMap.digitalChannel.get("touch");
        
        //REVERSES these motors for proper driving
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        
        
        /*
        FunctionsForAutonomous contains all our methods for robot automation
        during the autonomous round. Here we create an DriveAutonomous 
        object named "robot", which we will use to call methods.
        */
        FunctionsForAutonomous robot = new FunctionsForAutonomous(motorFrontRight, motorFrontLeft,
        motorBackRight, motorBackLeft, liftMotor, bucketServo, touchSensor);
        
        waitForStart();
        
        //robot.resetBucket();
        //robot.land();
        robot.moveSide(12);
        robot.turn(90);
        robot.move(24);
        
    }
    
}
