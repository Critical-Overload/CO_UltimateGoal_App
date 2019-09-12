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

        //Create an IMURobot object that we will use to run the robot
        IMURobot robot = new IMURobot(motorFrontRight, motorFrontLeft, motorBackRight, motorBackLeft, imu, this);

        robot.setupRobot();//calibrate IMU, set any required parameters

        waitForStart(); //wait for the game to start

        robot.gyroTurn(90, 0.3); //turn 90 degrees counterclockwise
        sleep(500);
        robot.gyroTurn(-90, 0.3); //turn 120 degrees clockwise
        sleep(500); //wait
        robot.gyroDrive(0.3, 5);//go forward with gyro for 5 seconds
        sleep(500);//wait
        robot.tankDrive(0.3, 0.3);//set power to motors
        sleep(5000);//drive for 5 seconds
        robot.completeStop();//stop
    }
}
