package org.firstinspires.ftc.teamcode.old;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="AutonomousDepot")

public class AutonomousDepot extends LinearOpMode {
    
    //declares motors and servos
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
        //set driving motors to encoder run-to-position mode
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        */
        /*
        FunctionsForAutonomous contains all our methods for robot automation
        during the autonomous round. Here we create an DriveAutonomous 
        object named "robot", which we will use to call methods.
        */
        FunctionsForAutonomous robot = new FunctionsForAutonomous(motorFrontRight, motorFrontLeft,
        motorBackRight, motorBackLeft, liftMotor, bucketServo, touchSensor);
        
        //for REV Hex motors
        int ticksCycle=288;
        
        waitForStart();
        robot.fullPower();
        //robot reset Bucket 
        //robot land
        robot.turn(90);
        robot.move(23.5*Math.sqrt(2)/2);
        //SCAN
        //if left;
        robot.moveSide(23.5*Math.sqrt(2)/2);
        robot.move(23.5*Math.sqrt(2));
        robot.moveSide(-23.5*Math.sqrt(2)/2);
            //dump arm, bucket
        robot.moveSide(-23.5*Math.sqrt(2)/2);
        robot.turn(45);
        robot.move(-3.5*23.5);
        //if center;
            //robot.move(23.5*Math.sqrt(2));
            //dump arm, bucket
            //robot.moveSide(-23.5*Math.sqrt(2)/2);
            //robot.turn(45);
            //robot.move(-3.5*23.5);
        //if right;
            //robot.moveSide(-23.5*Math.sqrt(2)/2);
            //robot.move(23.5*Math.sqrt(2));
            //robot.turn(45);
            //dump arm, bucket
            //robot.move(-3.5*23.5);
    }
}
