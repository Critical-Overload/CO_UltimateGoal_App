package org.firstinspires.ftc.teamcode.old;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "LanderLiftAttempt1")

public class LanderLiftTrial1 extends LinearOpMode{

    private DcMotor liftMotor;
        
    @Override
    public void runOpMode(){

        liftMotor = hardwareMap.dcMotor.get("liftMotor");
        liftMotor.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while(opModeIsActive()){
            if(gamepad1.dpad_up) {

                liftMotor.setPower(-0.5);

            }else if(gamepad1.dpad_down){

                liftMotor.setPower(0.5);

            }else{

                liftMotor.setPower(0);

        }
        idle();

    }

  }
        
    
}