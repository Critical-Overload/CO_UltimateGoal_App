package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class FoundationMovingTeleOp extends LinearOpMode {

    //Declare motors and servos
    private DcMotor motorLeft;
    private DcMotor motorRight;

    private Servo leftServo;
    private Servo rightServo;

    public void runOpMode() throws InterruptedException{

        //Initialize motors and servos
        motorLeft = hardwareMap.dcMotor.get("left");
        motorRight = hardwareMap.dcMotor.get("right");

        leftServo = hardwareMap.servo.get("l");
        rightServo = hardwareMap.servo.get("r");

        leftServo.setPosition(0.8);
        rightServo.setPosition(0.2);

        while(opModeIsActive()){
            motorLeft.setPower(gamepad1.left_stick_y);
            motorRight.setPower(gamepad1.right_stick_y);

            if(gamepad1.dpad_up){
                leftServo.setPosition(leftServo.getPosition() + 0.01);
                rightServo.setPosition(rightServo.getPosition() - 0.01);
            }else if(gamepad1.dpad_down){
                leftServo.setPosition(leftServo.getPosition() - 0.01);
                rightServo.setPosition(rightServo.getPosition() + 0.01);
            }
        }
    }
}
