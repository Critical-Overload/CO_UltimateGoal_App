package org.firstinspires.ftc.teamcode.old;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


//The Label @Autonomous means it will appear in the Autonomous drop-down list when you open the
// driver station.
@Autonomous(name = "Jolie Autonomous Test2")
//You can insert @Disabled if you don't want it to appear when you are choosing an Op mode.
public class JolieAutonomousTest2 extends LinearOpMode
{
    //Declare Motors
    private DcMotor motorFrontRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackRight;
    private DcMotor motorBackLeft;

    @Override

    public void runOpMode() throws InterruptedException
    {
        //Initialize motors
        motorBackLeft = hardwareMap.dcMotor.get("1");
        motorBackRight = hardwareMap.dcMotor.get("0");
        motorFrontLeft = hardwareMap.dcMotor.get("3");
        motorFrontRight = hardwareMap.dcMotor.get("2");

        //One Motor from each pair has to be reversed so
        //that the robot doesn't spin in a circle.
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);

        //Wait for Start button to be pressed.
        waitForStart();


        DriveRightTime(0.5,500);
        DriveDiagonalRightBackwardTime(0.5,500);
        DriveBackwardTime(0.5,500);
        DriveDiagonalLeftBackwardTime(0.5,500);
        DriveLeftTime(0.5,500);
        DriveDiagonalLeftForwardTime(0.5,500);
        DriveForwardTime(0.5,500);
        DriveDiagonalRightForwardTime(0.5,500);
        Stop();

    }

    //Methods (like functions in Python)
    private void DriveRight(double power)
    {
        motorBackRight.setPower(power);
        motorFrontRight.setPower(power);
    }
    private void DriveDiagonalRightBackward(double power)
    {
        DriveBackward(power);
        DriveRight(power);
    }
    private void DriveBackward(double power)
    {
        motorFrontLeft.setPower(-power);
        motorBackLeft.setPower(-power);
    }
    private void DriveDiagonalLeftBackward(double power)
    {
        DriveBackward(power);
        DriveLeft(power);
    }
    private void DriveLeft(double power)
    {
        DriveRight(-power);
    }
    private void DriveDiagonalLeftForward(double power)
    {
        DriveLeft(power);
        DriveForward(power);
    }
    private void DriveForward(double power)
    {
        DriveBackward(-power);
    }
    private void DriveDiagonalRightForward(double power)
    {
        DriveRight(power);
        DriveForward(power);
    }
    private void DriveRightTime(double power, long time) throws InterruptedException
    {
        DriveRight(power);
        Thread.sleep(time);
    }
    private void DriveDiagonalRightBackwardTime(double power, long time) throws InterruptedException
    {
        DriveDiagonalRightBackward(power);
        Thread.sleep(time);
    }
    private void DriveBackwardTime(double power, long time) throws InterruptedException
    {
        DriveBackward(power);
        Thread.sleep(time);
    }
    private void DriveDiagonalLeftBackwardTime(double power, long time) throws InterruptedException
    {
        DriveDiagonalLeftBackward(power);
        Thread.sleep(time);
    }
    private void DriveLeftTime(double power, long time) throws InterruptedException
    {
        DriveLeft(power);
        Thread.sleep(time);
    }
    private void DriveDiagonalLeftForwardTime(double power, long time) throws InterruptedException
    {
        DriveDiagonalLeftForward(power);
        Thread.sleep(time);
    }
    private void DriveForwardTime(double power, long time) throws InterruptedException
    {
        DriveForward(power);
        Thread.sleep(time);
    }
    private void DriveDiagonalRightForwardTime(double power, long time) throws InterruptedException
    {
        DriveDiagonalRightForward(power);
        Thread.sleep(time);
    }
    public void TurnClockwise(double power)
    {
       motorFrontLeft.setDirection(DcMotor.Direction.FORWARD);

    }

    private void Stop()
    {
        motorBackLeft.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackRight.setPower(0);
        motorFrontRight.setPower(0);
    }



}
