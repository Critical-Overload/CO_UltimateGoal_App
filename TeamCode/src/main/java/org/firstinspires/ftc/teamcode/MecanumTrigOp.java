package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

//Created October 20, 2018 by Jonathan

/* 
Uses Java built in trig functions to calculate motor powers
gamepad 1 right stick controls direction
gamepad 1 left stick controls rotation
*/

@TeleOp(name = "MecanumTrigOp")
public class MecanumTrigOp extends LinearOpMode
{
    private DcMotor motorFrontRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackRight;
    private DcMotor motorBackLeft;

    @Override
    public void runOpMode () throws InterruptedException
    {
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

            double angle = Math.atan2(gamepad1.right_stick_y, gamepad1.right_stick_x) - 3*(Math.PI)/4;
            double r = Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y);
            double rotation = gamepad1.left_stick_x;
            
            motorFrontLeft.setPower((r * Math.cos(angle) + rotation)*powerMod);
            motorBackRight.setPower((r * Math.cos(angle) - rotation)*powerMod);
            motorFrontRight.setPower((r * Math.sin(angle) - rotation)*powerMod);
            motorBackLeft.setPower((r * Math.sin(angle) + rotation)*powerMod);
            

            idle();
        }
    }
}
