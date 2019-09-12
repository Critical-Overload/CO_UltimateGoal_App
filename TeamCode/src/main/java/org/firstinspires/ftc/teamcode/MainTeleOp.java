package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.logging.Level;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

//Created October 28, 2018 by Jonathan
//Authors: Jonathan and Robert

/* 
Uses Mecanum Trigonometric Driving
Incorporates Lift Motor
*/

@TeleOp(name = "MainTeleOp")
public class MainTeleOp extends LinearOpMode
{
    //Init motors and servos
    private DcMotor motorFrontRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackRight;
    private DcMotor motorBackLeft;
    private DcMotor liftMotor;
    private DcMotor hingeMotor;
    private Servo bucketServo;
    private CRServo intakeServo;
    private DigitalChannel touchSensor;
    

    @Override
    public void runOpMode () throws InterruptedException
    {
        //Define motors and servos
        motorFrontRight = hardwareMap.dcMotor.get("FR");
        motorFrontLeft = hardwareMap.dcMotor.get("FL");
        motorBackRight= hardwareMap.dcMotor.get("BR");
        motorBackLeft = hardwareMap.dcMotor.get("BL");
        liftMotor = hardwareMap.dcMotor.get("lift");
        hingeMotor = hardwareMap.dcMotor.get("hingeMotor");
        bucketServo = hardwareMap.servo.get("bucketServo");
        intakeServo = hardwareMap.crservo.get("intakeServo");
        touchSensor = hardwareMap.digitalChannel.get("touchSensor");

        /*These motors need to have reversed directions
        Because of how they are placed on the robot*/
        
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);
        //hingeMotor.setDirection(DcMotor.Direction.REVERSE);
        
        //set lift and hinge motors to encoder run-to-position mode
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hingeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hingeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //configure lift variables
        int liftLevel=0;
        int liftTicks=0;
        int ticksCycle=1440;
        
        //configure bucket position variable
        double bp = 1.0;

        //powerMod variables can reduce robot speed
        double powerMod = 1.0;
        double mechMod = 1.0;
        
        //servo parameters
        bucketServo.scaleRange(.1,1);
        
        telemetry.addData("Hey", "Listen");
        telemetry.addData("Level of lift: ", liftLevel);
        telemetry.addData("Encoder Position ", liftMotor.getCurrentPosition());
        telemetry.addData("Lift Ticks ", liftTicks);
        telemetry.addData("Touch Sensor", "Ready");
        telemetry.update();

        waitForStart();
        //set starting power for encoded motors
        liftMotor.setPower(1);
        double maxH = 1; //maximum power for hinge motor
        hingeMotor.setPower(maxH);
        
        hingeMotor.setTargetPosition(0); //prevents weird error of going to random #
        
        while(opModeIsActive())
        {
            
            //vary hinge power based on position
            int hp = hingeMotor.getCurrentPosition(); //gets position as short-named variable for use (also used for bucket positioning)
            int htp = hingeMotor.getTargetPosition();
            double hpercent = 1;
            
            if(htp == (3*ticksCycle) && hp >= (1.2*ticksCycle) && hp <= (3.0*ticksCycle)){
                if(hp>(2.3*ticksCycle)){
                    hpercent = 0.5;
                }
                else{
                    hpercent = (Math.abs(htp-hp)/(1.8*ticksCycle));
                }
                if(hpercent < 0.001){ //percent of max power deemed minimum to run
                    hpercent = 0.001;
                }
                double power = maxH*hpercent;
                hingeMotor.setPower(power);
            }
            
            
            //add comment about if condition here
            else if((hp >= (htp-(0.5*ticksCycle))) && (hp < (htp+(0.5*ticksCycle)))){
                hpercent = (Math.abs(htp-hp)/(0.5*ticksCycle));
                if(hpercent < 0.001){ //percent of max power deemed minimum to run
                    hpercent = 0.001;
                }
                telemetry.addData("hpercent", hpercent);
                double power = maxH * hpercent;
                if(gamepad2.right_stick_y == 0){
                    hingeMotor.setPower(power);
                }else{
                    hingeMotor.setPower(maxH);
                }
            }
            else{
                hingeMotor.setPower(maxH);
            }
            telemetry.addData("hingePower", hingeMotor.getPower());
            /*if(hp >= 0 && hp < (0.4*ticksCycle)){
                    hpercent = hp/(0.4*ticksCycle);
                    if(hpercent < 0.2){ //percent of max power deemed minimum to run
                        hpercent = 0.2;
                    }
                }
            else if(hp >= (2.6*ticksCycle) && hp <= (3*ticksCycle)){
                    hpercent = ((3*ticksCycle)-hp)/(0.4*ticksCycle);
                    if(hpercent < 0.2){ //percent of max power deemed minimum to run
                        hpercent = 0.2;
                    }
                }
            double power = maxH * hpercent;
            hingeMotor.setPower(power);
            */

            /*
            Checks if right bumper is pressed.
            If so, power is reduced.
             */
            if(gamepad1.right_bumper){
                powerMod = 0.5;
            }
            else{
                powerMod = 1.0;
            }
            if(gamepad2.right_bumper){
                mechMod = 0.5;
            }
            else{
                mechMod = 1.0;
            }
            
            //move lift
            if (gamepad1.dpad_left || gamepad1.dpad_right) {
                liftTicks = (int)((5.75*ticksCycle)/2);
                liftLevel = 1;
                liftMotor.setTargetPosition(liftTicks);
            }    
            else if (gamepad1.dpad_up) {
                liftTicks = 5800;
                liftLevel = 2;
                liftMotor.setTargetPosition(liftTicks);
            } 
            else if (gamepad1.dpad_down){
                liftTicks = 0;
                liftLevel = 0;
                liftMotor.setTargetPosition(liftTicks);
            }
            if (gamepad1.left_bumper){
                liftTicks -= 10*powerMod;
                liftMotor.setTargetPosition(liftTicks);
            }
            if (gamepad1.x){
                liftTicks += 10*powerMod;
                liftMotor.setTargetPosition(liftTicks);
            }
            

            /*
            Mecanum wheel drive using trigonometry
            */
            double angle = Math.atan2(gamepad1.right_stick_y, gamepad1.right_stick_x) + (Math.PI)/4;
            double r = Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y);
            double rotation = gamepad1.left_stick_x *0.5;//halved for control
            
            motorFrontLeft.setPower((r * Math.cos(angle) - rotation)*powerMod);
            motorBackRight.setPower((r * Math.cos(angle) + rotation)*powerMod);
            motorFrontRight.setPower((r * Math.sin(angle) - rotation)*powerMod);
            motorBackLeft.setPower((r * Math.sin(angle) + rotation)*powerMod);
            
            //controls bucketservo (REMOVED:WILL BE REPLACED BY ARM-SENSING AUTOMATION)
            /*if(gamepad2.a){
                bucketServo.setPosition(0);
            }else if(gamepad2.b){
                bucketServo.setPosition(1);
            }else if(gamepad2.left_bumper){
                bucketServo.setPosition(0.4);
            }*/
            
            //automatically set bucket servo to optimized position
            double bpercent = 0;
            if(!gamepad2.left_bumper){
                if(hp >= 0 && hp < (1.2*ticksCycle)){
                    //set between 1 and .4
                    bpercent = hp/(1.2*ticksCycle);
                    bp = 1-(bpercent*(1-.4));
                }
                else if(hp >= (1.2*ticksCycle) && hp <= (3*ticksCycle)){
                    //set between .4 and .0
                    bpercent = (hp - 1.2*ticksCycle)/(3*ticksCycle-1.2*ticksCycle);
                    bp = .4-(bpercent*(.4-.0));
                }

                bucketServo.setPosition(bp);
            }
            
            /*//breakage prevention
            if(hingeMotor.getCurrentPosition() > 3*ticksCycle){
                hingeMotor.setTargetPosition(hingeMotor.getCurrentPosition());
            }*/
            /*
            //dump bucket
            if((gamepad2.dpad_left || gamepad2.dpad_right) && gamepad2.left_bumper){
                bucketServo.setPosition(0.36);
            }*/

            //intake:spins in intake direction unless override button (x) pressed
            if(gamepad2.x){
            intakeServo.setPower(Math.abs(gamepad2.left_trigger));
            }
            else{
            intakeServo.setPower(-Math.abs(gamepad2.left_trigger));
            }
            
            //manual bucket positioning
            if(gamepad2.dpad_up){
                bucketServo.setPosition(bucketServo.getPosition()+0.01);
            }
            else if(gamepad2.dpad_down){
                bucketServo.setPosition(bucketServo.getPosition()-0.01);
            }
            
            //Arm movement
            if(gamepad2.right_stick_y!=0){
                hingeMotor.setTargetPosition((int)(hingeMotor.getCurrentPosition()-(gamepad2.right_stick_y*120*mechMod)));
                telemetry.addData("right_stick",(int)(hingeMotor.getCurrentPosition()-(gamepad2.right_stick_y*120*mechMod)));
            }
            else if(gamepad2.a){
                hingeMotor.setTargetPosition(0);
                if(touchSensor.getState() == false){
                    hingeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    Thread.sleep(1000);
                    hingeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    //hingeMotor.setTargetPosition(0);
                }
                telemetry.addData("A", "Pressed");
            }
            else if(gamepad2.b){
                hingeMotor.setTargetPosition((int)(1.2*ticksCycle));
                telemetry.addData("set B: ", (int)(1.2*ticksCycle));
            }
            else if(gamepad2.y){
                hingeMotor.setTargetPosition((int)(3*ticksCycle));
                telemetry.addData("set Y: ", (int)(3*ticksCycle));
            }
            if(gamepad2.right_stick_button){
                hingeMotor.setTargetPosition(hingeMotor.getCurrentPosition());
                telemetry.addData("set RSB: ", hingeMotor.getCurrentPosition());
            }
            
            //DEBUGGING:insert after removing all other arm controls to 
            //move arm to starting position
            /*if (gamepad2.right_stick_button){
                hingeMotor.setPower(-1);
            }*/
            
            
            //if the touch sensor is pressed, then stop motors
            if(touchSensor.getState() == false){
                    telemetry.addData("Touch Sensor","Pressed");
                }
                
                
                //NEED TO READD TOUCH SENSOR BY ZEROING ENCODERS hingeMotor
            
            else{telemetry.addData("Touch Sensor","Not pressed");}
            
            //telemetry.addData("Hey", "Listening");
            telemetry.addData("Level of lift: ", liftLevel);
            telemetry.addData("Encoder Position ", liftMotor.getCurrentPosition());
            telemetry.addData("Lift Ticks ", liftTicks);
            telemetry.addData("Bucket position", bucketServo.getPosition());
            telemetry.addData("Arm Position: ", hingeMotor.getCurrentPosition());
            telemetry.addData("Arm Target: ", hingeMotor.getTargetPosition());
            telemetry.addData("Arm Mode: ", hingeMotor.getMode());
            telemetry.update();

            idle();
        }
    }
}
