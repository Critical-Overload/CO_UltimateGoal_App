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

        motorBackRight.setDirection(DcMotor.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);



        double powerMod = 1.0;
        double speedCal = 0.5;

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

            double angle = Math.atan2(gamepad1.right_stick_y, gamepad1.right_stick_x) - (Math.PI)/4;
            double r = Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y);
            double rotation = gamepad1.left_stick_x;

            double powerOne = r * Math.cos(angle)*powerMod;
            double powerTwo = r * Math.sin(angle)*powerMod;

            if (powerOne < 0.8 && powerOne > -0.8){
                speedCal = 0.6;
            }else{
                speedCal = 0.49;
            }
            
            motorFrontLeft.setPower((powerOne - rotation)*powerMod);
            motorBackRight.setPower((powerOne + rotation)*powerMod*speedCal);
            motorFrontRight.setPower((powerTwo + rotation)*powerMod);
            motorBackLeft.setPower((powerTwo - rotation)*powerMod*speedCal);

            telemetry.addData("power1", powerOne);
            telemetry.addData("power2", powerTwo);

            telemetry.update();
            idle();
        }
    }
}
