package org.firstinspires.ftc.teamcode.old;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/*
Test for best driving control
Steer Drive:
Left Stick - Controls Left Side of Robot (Forward and Backwards)
Right Stick - Controls Right Side of Robot (Forward and Backwards)
 */

@TeleOp(name = "MecanumWheelTest")
public class MecanumWheelTest extends LinearOpMode
{
    private DcMotor motorFrontRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackRight;
    private DcMotor motorBackLeft;

    @Override
    public void runOpMode () throws InterruptedException
    {
        motorFrontRight = hardwareMap.dcMotor.get("0");
        motorFrontLeft = hardwareMap.dcMotor.get("2");
        motorBackRight= hardwareMap.dcMotor.get("1");
        motorBackLeft = hardwareMap.dcMotor.get("3");

        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);


        double powerMod = 1.0;

        waitForStart();

        while(opModeIsActive())
        {
            
            motorFrontRight.setPower(gamepad1.right_stick_y);
            motorFrontLeft.setPower(gamepad1.left_stick_y);
            motorBackRight.setPower(gamepad2.right_stick_y);
            motorBackLeft.setPower(gamepad2.left_stick_y);
            

            idle();
        }
    }
}