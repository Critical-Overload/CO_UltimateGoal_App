package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import java.math.BigDecimal;

@TeleOp(name = "AAMainTeleOp")
public class MainTeleOp extends LinearOpMode {

    private DcMotor motorFrontRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackLeft;
    private DcMotor motorBackRight;
    private DcMotor arm;
    private CRServo leftIntake;
    private CRServo rightIntake;

    private Servo leftIntakeServo;
    private Servo rightIntakeServo;
    private Servo flimsy;
    private Servo clawMover;
    private Servo claw;
    private Servo capRelease;


    double ap;
    double ClawPosition;

    private DigitalChannel touch;

    @Override
    public void runOpMode() throws InterruptedException {

        motorFrontRight = hardwareMap.dcMotor.get("FR");
        motorFrontLeft = hardwareMap.dcMotor.get("FL");
        motorBackLeft = hardwareMap.dcMotor.get("BL");
        motorBackRight = hardwareMap.dcMotor.get("BR");
        arm = hardwareMap.dcMotor.get("arm");

        leftIntake = hardwareMap.crservo.get("LI");
        rightIntake = hardwareMap.crservo.get("RI");

        leftIntakeServo = hardwareMap.servo.get("LIrelease");
        rightIntakeServo = hardwareMap.servo.get("RIrelease");
        flimsy = hardwareMap.servo.get("flimsy");
        clawMover = hardwareMap.servo.get("clawMover");
        claw = hardwareMap.servo.get("claw");
        capRelease = hardwareMap.servo.get("capRelease");



        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);
        leftIntake.setDirection(CRServo.Direction.REVERSE);

        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);





        double powerMod = 1.0;
        double intakeMod = 1.0;

        waitForStart();


        while(opModeIsActive()){
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            /*
            Checks if right bumper is pressed. If so, power is reduced
             */
            if(gamepad1.right_bumper){
                powerMod = 0.5;
            }else{
                powerMod = 1.0;
            }

            if(gamepad2.left_bumper){
                intakeMod = 0.5;
            }else{
                intakeMod = 1.0;
            }

            if(gamepad2.a){
                leftIntakeServo.setPosition(0);
                rightIntakeServo.setPosition(1);
            }

            if(gamepad2.b){
                leftIntakeServo.setPosition(1);
                rightIntakeServo.setPosition(0);
            }

            if(gamepad2.dpad_up){
                leftIntakeServo.setPosition(leftIntakeServo.getPosition() + 0.01);
                rightIntakeServo.setPosition(rightIntakeServo.getPosition() - 0.01);

            }

            if(gamepad2.dpad_down){
                leftIntakeServo.setPosition(leftIntakeServo.getPosition() - 0.01);
                rightIntakeServo.setPosition(rightIntakeServo.getPosition() + 0.01);
            }

            if(gamepad2.x){
                claw.setPosition(0);
            }


            if(gamepad2.y){
                claw.setPosition(1);
            }

            if(gamepad2.right_stick_y != 0){
                ap = arm.getCurrentPosition();
                //clawMover.setPosition(ap/537.6);

            }

            if(gamepad2.right_bumper){
                arm.setTargetPosition(-50);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(0.2);
                while(arm.isBusy()){
                    clawMover.setPosition(0.1);
                }
                arm.setPower(0);
            }

            if (gamepad2.left_bumper){
                arm.setTargetPosition(650);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(0.2);
                while (arm.isBusy()){
                    sleep(500);
                    clawMover.setPosition(0.4);
                }
                arm.setPower(0);
            }


            if(gamepad1.a){
                flimsy.setPosition(flimsy.getPosition() + 0.01);
            }
            if(gamepad1.b){
                flimsy.setPosition(flimsy.getPosition() - 0.01);
            }

            arm.setPower(gamepad2.right_stick_y);
            while (gamepad2.right_stick_y != 0){
                double Postion = arm.getCurrentPosition();
                double ClawPosition = Postion/1400;
                clawMover.setPosition(ClawPosition);
            }

            if(gamepad1.left_bumper){
                capRelease.setPosition(1);
            }

            arm.setTargetPosition(arm.getCurrentPosition());


            clawMover.setPosition(clawMover.getPosition()+gamepad2.left_stick_y*0.01);



            telemetry.addData("Flimsy servo position", flimsy.getPosition());
            telemetry.addData("Arm Encoder Number", arm.getCurrentPosition());
            telemetry.addData("Claw Mover Postion", ClawPosition);

            leftIntake.setPower((gamepad2.right_trigger * intakeMod)-(gamepad2.left_trigger * intakeMod));
            rightIntake.setPower((gamepad2.right_trigger * intakeMod)-(gamepad2.left_trigger * intakeMod));

            double angle = Math.atan2(gamepad1.right_stick_y, gamepad1.right_stick_x) - (Math.PI/4);
            double r = Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y);
            double rotation = gamepad1.left_stick_x;

            double powerOne = r*Math.sin(angle);
            double powerTwo = r*Math.cos(angle);

            motorFrontLeft.setPower((powerOne - (rotation))*powerMod);
            motorFrontRight.setPower((powerTwo + (rotation))*powerMod);
            motorBackLeft.setPower((powerTwo - (rotation))*powerMod);
            motorBackRight.setPower((powerOne + (rotation))*powerMod);

            telemetry.addData("FL Power", motorFrontLeft.getPower());
            telemetry.addData("BL Power", motorBackLeft.getPower());
            telemetry.addData("FR Power", motorFrontRight.getPower());
            telemetry.addData("BR Power", motorBackRight.getPower());

            telemetry.update();
            idle();

        }
    }
}
