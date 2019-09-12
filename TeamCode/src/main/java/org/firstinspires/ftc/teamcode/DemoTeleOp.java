//This package contains all the libraries we need to program the FTC Robot Controller
package org.firstinspires.ftc.teamcode;

//Import all the classes we will need
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;/*
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;*/

//TeleOp stands for the driver controlled portion of the robot game
//@TeleOp adds the program to the TeleOp section of the Robot Controller App with the name "DemoTeleOp"
@TeleOp(name = "DemoTeleOp")

//There are two types of OpModes, OpMode and LinearOpMode. We use LinearOpMode.
public class DemoTeleOp extends LinearOpMode {

    //Here, we initialize the motors we are using in the demonstration
    private DcMotor motorOne;
    private DcMotor motorTwo;
    /*private DcMotor motorThree;
    private DcMotor motorFour;

    //Initialize a servo
    private Servo servoOne;
    //Initialize a Continuous Rotation Servo. CRServos behave similar to motors
    private CRServo servoTwo;

    //Initialize a touch sensor. Touch sensors are DigitalChannel devices
    private DigitalChannel touchSensor;*/

    //Overrides the method runOpMode from superclass LinearOpMode
    @Override
    //runOpMode() is the function that is run when the program is initialized by the user
    //InterruptedException allows for the program to stop when stop is pressed
    public void runOpMode()throws InterruptedException{

        //Define all our motors, servos, and sensors using the hardware map created by our configuration
        motorOne = hardwareMap.dcMotor.get("motorOne");
        motorTwo = hardwareMap.dcMotor.get("motorTwo");
        /*motorThree = hardwareMap.dcMotor.get("motorThree");
        motorFour = hardwareMap.dcMotor.get("motorFour");

        servoOne = hardwareMap.servo.get("servoOne");
        servoTwo = hardwareMap.crservo.get("servoTwo");

        touchSensor = hardwareMap.digitalChannel.get("touchSensor");

        //Reverse motorThree
        motorThree.setDirection(DcMotor.Direction.REVERSE);

        //Motors can run to a position using the motor encoders. First, set the motor to run to position mode
        //and then reset the encoder for accuracy
        motorFour.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFour.setMode(DcMotor.RunMode.RUN_TO_POSITION);*/

        //Telemetry is useful for adding information to the printout on the app
        //First, data must be added to the telemetry log
        telemetry.addData("Status", "Ready");
        //Push telemetry data to be displayed
        telemetry.update();

        //Wait until start is pressed by the user
        waitForStart();/*

        //Set the power motorFour will run at and keep it in the start position
        motorFour.setPower(1);
        motorFour.setTargetPosition(0);*/

        //This loop will keep running until stop is pressed. Everything that happens during the run will be in this loop
        while(opModeIsActive()){
            motorOne.setPower(gamepad1.left_stick_y);
            motorTwo.setPower(gamepad1.right_stick_y);

        }
    }
}
