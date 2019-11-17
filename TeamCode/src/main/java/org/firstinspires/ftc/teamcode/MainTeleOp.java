package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "MainTeleOp")
public class MainTeleOp extends LinearOpMode {

    private DcMotor motorFrontRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackLeft;
    private DcMotor motorBackRight;
    private DcMotor leftIntake;
    private DcMotor rightIntake;
    private DcMotor arm;

    private Servo leftIntakeServo;
    private Servo rightIntakeServo;
    private Servo claw;

    @Override
    public void runOpMode() throws InterruptedException {

        motorFrontRight = hardwareMap.dcMotor.get("FR");
        motorFrontLeft = hardwareMap.dcMotor.get("FL");
        motorBackLeft = hardwareMap.dcMotor.get("BL");
        motorBackRight = hardwareMap.dcMotor.get("BR");
        leftIntake = hardwareMap.dcMotor.get("LI");
        rightIntake = hardwareMap.dcMotor.get("RI");
        arm = hardwareMap.dcMotor.get("arm");

        leftIntakeServo = hardwareMap.servo.get("LIservo");
        rightIntakeServo = hardwareMap.servo.get("RIservo");
        claw = hardwareMap.servo.get("claw");

        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);
        rightIntake.setDirection(DcMotor.Direction.REVERSE);

        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double powerMod = 1.0;
        double intakeMod = 1.0;

        waitForStart();

        claw.setPosition(0);

        while(opModeIsActive()){

            /*
            Checks if right bumper is pressed. If so, power is reduced
             */
            if(gamepad1.right_bumper){
                powerMod = 0.5;
            }else{
                powerMod = 1.0;
            }

            if(gamepad1.left_bumper){
                intakeMod = -1.0;
            }else{
                intakeMod = 1.0;
            }

            if(gamepad2.a){
                leftIntakeServo.setPosition(1);
                rightIntakeServo.setPosition(0);
            }

            arm.setPower(gamepad2.right_stick_y);

            if(gamepad2.b){
                claw.setPosition(1);
            }else if(gamepad2.x){
                claw.setPosition(0);
            }

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
            idle();
        }
    }
}
