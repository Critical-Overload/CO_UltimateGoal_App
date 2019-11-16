/**
 * IMURobotTest
 *
 * 30 March 2019
 */

package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.IMURobot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "IMURobotTest")
public class IMURobotTest extends LinearOpMode {
    //Declare motors
    private DcMotor motorFrontRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackRight;
    private DcMotor motorBackLeft;

    //Declare imu
    private BNO055IMU imu;

    public void runOpMode() throws InterruptedException{
        //Initialize motors
        motorFrontRight = hardwareMap.dcMotor.get("FR");
        motorFrontLeft = hardwareMap.dcMotor.get("FL");
        motorBackRight = hardwareMap.dcMotor.get("BR");
        motorBackLeft = hardwareMap.dcMotor.get("BL");
        //Initialize imu
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        //Reverse requred motors
        //motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        //motorBackRight.setDirection(DcMotor.Direction.REVERSE);
        //Set zero power behaviors to brake
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Create an IMURobot object that we will use to run the robot
        IMURobot robot = new IMURobot(motorFrontRight, motorFrontLeft, motorBackRight, motorBackLeft, imu, this);
        robot.setupRobot();//calibrate IMU, set any required parameters

        waitForStart(); //wait for the game to start


/*
        robot.gyroStrafeCont(.25, 180);
        wait(2000);
        robot.contStop();

        robot.gyroStrafeCont(.25,90);
        robot.completeStop();

 */


        telemetry.addData("Current status", "Turning");
        telemetry.update();
        robot.gyroTurn(90, 0.3); //turn 90 degrees counterclockwise
        sleep(500);
        robot.gyroTurn(-90, 0.3); //turn 90 degrees clockwise
        sleep(500); //wait
        telemetry.addData("Current status", "driving");
        telemetry.update();
        robot.gyroDriveEncoder(0.5, 60);
        robot.gyroStrafeEncoder(0.5, 90,60);//go forward with gyro for 90 cm
        robot.gyroStrafeEncoder(0.5, -90, 60);
        sleep(500);//wait
        telemetry.addData("Current status", "strafing");
        telemetry.update();
        /*
        robot.gyroStrafeEncoder(0.5, 135, 60);//strafe right for 90 cm
        sleep(500);//wait
        robot.gyroStrafeEncoder(0.5, 225, 60); //strafe at a degree for 90 cm
        sleep(500);
        robot.gyroStrafeEncoder(0.5, 315, 60);
        robot.completeStop();//stop*/



    }
}
