package org.firstinspires.ftc.teamcode.old;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/*
Test for best driving control
Steer Drive:
Left Stick - Controls Left Side of Robot (Forward and Backwards)
Right Stick - Controls Right Side of Robot (Forward and Backwards)
 */

@TeleOp(name = "MecanumTest")
public class MecanumOldOp extends LinearOpMode
{
    //declare motors
    private DcMotor motorFrontRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackRight;
    private DcMotor motorBackLeft;

    @Override
    public void runOpMode () throws InterruptedException
    {
        //initialize motors
        motorFrontRight = hardwareMap.dcMotor.get("FR");
        motorFrontLeft = hardwareMap.dcMotor.get("FL");
        motorBackRight= hardwareMap.dcMotor.get("BR");
        motorBackLeft = hardwareMap.dcMotor.get("BL");

        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);


        double powerMod = 1.0;

        waitForStart();

        while(opModeIsActive())
        {
            /*
            Checks if right bumper is pressed.
            If so, power is reduced.
             */
            if(gamepad1.right_bumper){
                powerMod = 0.5;
            }else{
                powerMod = 1.0;
            }

            /*
            D-pad right turns the robot right.
            D-pad left turns the robot left
            */
            if(gamepad1.dpad_left){
                motorFrontRight.setPower(powerMod);
                motorBackLeft.setPower(powerMod);
                motorFrontLeft.setPower(-1*powerMod);
                motorBackRight.setPower(-1*powerMod);
            }else if(gamepad1.dpad_right){
                motorFrontLeft.setPower(powerMod);
                motorBackRight.setPower(powerMod);
                motorFrontRight.setPower(-1*powerMod);
                motorBackLeft.setPower(-1*powerMod);
            }else{
                /*
                Both sticks forward: Robot moves forward
                Both sticks backward: Robot moves backward
                Left stick forward and right stick backward: Robot moves left
                Right stick forward and left stick backward: Robot moves right
                */
                motorFrontLeft.setPower(gamepad1.left_stick_y*powerMod);
                motorBackLeft.setPower(gamepad1.left_stick_y*powerMod);
                motorFrontRight.setPower(gamepad1.right_stick_y*powerMod);
                motorBackRight.setPower(gamepad1.right_stick_y*powerMod);
            }
            

            idle();
        }
    }
}
