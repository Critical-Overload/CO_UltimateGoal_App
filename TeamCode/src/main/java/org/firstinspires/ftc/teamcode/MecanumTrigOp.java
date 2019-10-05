package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

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
    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation roAngle, lastAngles, startAngles;
    Acceleration gravity;
    private double globalAngle;
    private double gain = 0.1;



    @Override
    public void runOpMode () throws InterruptedException
    {
        motorFrontRight = hardwareMap.dcMotor.get("FR");
        motorFrontLeft = hardwareMap.dcMotor.get("FL");
        motorBackRight= hardwareMap.dcMotor.get("BR");
        motorBackLeft = hardwareMap.dcMotor.get("BL");


        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double powerMod = 1.0;
        double speedCal = 0.5;
        double leftWheelMod = 1.0;
        double rightWheelMod = 1.0;
        double fakeLeftTurn = 1.0;
        double fakeRightTurn = 1.0;


        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

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
            if (gamepad1.dpad_left){
                fakeLeftTurn = 0.9;
            }else{
                fakeLeftTurn = 1.0;
            }
            if (gamepad1.dpad_right){
                fakeRightTurn = 0.9;
            }else{
                fakeRightTurn = 1.0;
            }
            double angle = (Math.atan2(gamepad1.right_stick_y, gamepad1.right_stick_x) + (Math.PI)/4);
            double r = Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y);
            double rotation = gamepad1.left_stick_x;
            roAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            startAngles = lastAngles;
            globalAngle = 0;
            double correction = getCorrection();

            double powerOne = r * Math.cos(angle)*powerMod/0.707106;
            double powerTwo = r * Math.sin(angle)*powerMod/0.707106;

            if (globalAngle > 0 && gamepad1.left_stick_x == 0){
                rightWheelMod = -(1-correction);
            }
            if (globalAngle < 0 && gamepad1.left_stick_x == 0){
                leftWheelMod = (1-correction);
            }
            motorFrontLeft.setPower(((powerOne + rotation)*leftWheelMod*powerMod*fakeLeftTurn));
            motorBackRight.setPower(((powerOne - rotation)*rightWheelMod*powerMod*fakeRightTurn));
            motorFrontRight.setPower(((powerTwo + rotation)*rightWheelMod*powerMod*fakeRightTurn));
            motorBackLeft.setPower(((powerTwo - rotation)*leftWheelMod*powerMod*fakeLeftTurn));

            telemetry.addData("power1", powerOne);
            telemetry.addData("power2", powerTwo);
            telemetry.addData("angle", globalAngle);
            telemetry.addData("LeftMod", leftWheelMod);
            telemetry.addData("RightMod",rightWheelMod);


            telemetry.update();
            idle();
        }
    }
    private double getAngle(){
        //Get a new angle measurement
        //Get the difference between current angle measurement and last angle measurement
        double deltaAngle = roAngle.firstAngle - lastAngles.firstAngle;

        //Process the angle to keep it within (-180,180)
        //(Once angle passes +180, it will rollback to -179, and vice versa)
        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        //Add the change in angle since last measurement (deltaAngle)
        //to the change in angle since last reset (globalAngle)
        globalAngle += deltaAngle;
        //Set last angle measurement to current angle measurement
        lastAngles = roAngle;

        return globalAngle;
    }
    private double getCorrection() {
        //Get the current angle of the robot
        double Rangle = getAngle();
        double correction;

        //Use the angle to calculate the correction
        if (Rangle == 0) {
            //If angle = 0, robot is moving straight; no correction needed
            correction = 0;
        } else {
            //If angle != 0, robot is not moving straight
            //Correction is negative angle (to move the robot in the opposite direction)
            //multiplied by gain; the gain is the sensitivity to angle
            //We have determined that .1 is a good gain; higher gains result in overcorrection
            //Lower gains are ineffective
            correction = -Rangle * gain;
        }
        return correction;
    }
}
