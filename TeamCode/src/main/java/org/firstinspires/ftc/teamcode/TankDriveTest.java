package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

//Created October 20, 2018 by Jonathan

/* 
Uses Java built in trig functions to calculate motor powers
gamepad 1 right stick controls direction
gamepad 1 left stick controls rotation
*/

@TeleOp(name = "TankDriveTest")
public class TankDriveTest extends LinearOpMode
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



        double powerMod = 1.0;

        waitForStart();

        while(opModeIsActive())
        {
            /*
            Checks if right bumper is pressed.
            If so, power is reduced.
             */
            if(gamepad1.right_bumper){
                powerMod = 0.4;
            }else{
                powerMod = 0.8;
            }
            
            motorFrontLeft.setPower(-gamepad1.left_stick_y*powerMod);
            motorBackLeft.setPower(-gamepad1.left_stick_y*0.375*powerMod);
            motorBackRight.setPower(gamepad1.right_stick_y*0.375*powerMod);
            motorFrontRight.setPower(gamepad1.right_stick_y*powerMod);

            idle();
        }
    }
}
