package org.firstinspires.ftc.teamcode.old;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp(name = "BucketTest")

//Tests the bucket used to input and output mechanisms

public class BucketTest extends LinearOpMode{

    //declare motors and servos
    private DcMotor hingeMotor;
    private Servo bucketServo;
    private CRServo intakeServo;
    
    public void runOpMode() throws InterruptedException {
        
        //initialize motors and servos
        hingeMotor = hardwareMap.dcMotor.get("hingeMotor");
        bucketServo = hardwareMap.servo.get("bucketServo");
        intakeServo = hardwareMap.crservo.get("intakeServo");
        
        hingeMotor.setDirection(DcMotor.Direction.REVERSE);
        
        waitForStart();
        
        double powerMod = 0;
        
        bucketServo.setPosition(0.0);
        
        while(opModeIsActive()){
            
            if(gamepad1.a){
                bucketServo.setPosition(-1);
            }else if(gamepad1.b){
                bucketServo.setPosition(1);
            }
            
            if(gamepad1.x){
                intakeServo.setPower(0.75);
            }else if(gamepad1.y){
                intakeServo.setPower(-0.75);
            }else{
                intakeServo.setPower(0);
            }
            
            hingeMotor.setPower(gamepad1.right_stick_y);
            
            telemetry.update();
            
            idle();
        }
    }
    
}