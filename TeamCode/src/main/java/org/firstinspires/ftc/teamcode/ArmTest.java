package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ArmTest")
public class ArmTest extends LinearOpMode {

    //configure motors and servos
    private DcMotor arm;
    private Servo armServo;
    private Servo claw;

    public void runOpMode() throws InterruptedException {

        //Initialize motors and servos
        arm = hardwareMap.dcMotor.get("arm");
        armServo = hardwareMap.servo.get("armServo");
        claw = hardwareMap.servo.get("claw");
        double powerMod;

        //wait for start button to be pressed
        waitForStart();

        while(opModeIsActive()){

            //if right bumper is pressed, the robot will slow down
            if(gamepad1.right_bumper){
                powerMod = 0.25;
            }else{
                powerMod = 0.5;
            }


            //if a is pressed, the arm will turn to ____ position, if b is pressed, arm will turn to ____ position
            if(gamepad1.a){
                armServo.setPosition(0);
            }else if(gamepad1.b){
                armServo.setPosition(1);
            }


            //if x is pressed, the claw will ___, if y is pressed, the claw will ___
            if(gamepad1.x){
                claw.setPosition(0);
            }else if(gamepad1.y){
                claw.setPosition(1);
            }


            // the linear slide moves up and down if we move right stick
            arm.setPower(powerMod*gamepad1.right_stick_y);

            //a buffer
            idle();

        }
    }
}
