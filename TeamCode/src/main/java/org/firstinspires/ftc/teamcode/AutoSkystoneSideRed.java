/**
 * IMURobotTest
 *
 * 30 March 2019
 */
package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.IMURobot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

@Autonomous(name = "AutoSkystoneSideRed")
public class AutoSkystoneSideRed extends LinearOpMode {
    //Declare motors
    private DcMotor motorFrontRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackRight;
    private DcMotor motorBackLeft;

    private CRServo leftIntake;
    private CRServo rightIntake;

    private DcMotor arm;

    private Servo leftIntakeServo;
    private Servo rightIntakeServo;

    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    private static final String VUFORIA_KEY = "AeXjUy3/////AAABmbHLSgwTzU5tnItQeMYIKBuJtzLC3Yjgadn90RIg4wpjZJxXPoAwCZmsm+bAMXon60mNlk3UZvSNQaabvijg0UZ+9vB/U+d8CeHrLU4FziM5JseM/zIMAdJoePSg1sli9hlC1LYIPMd6uCYwuS8QZvkhHkBisttfhafsExbOSeIP/a3sBqDAxQ7rm1SIvWxdGAgu2iUrLVMx6affe6GAtcFMRhIVWLj+m6XyRnswbtS/Sh4hNZXoBTJn4py4rhZ4iouKw6SvZvYCokuqxObpBxG8Ni0pLcbtctO5+8xtb47Y4uwkkI9t7L4IUfntsQzaMcd6eMtoPcYIhLKhiydVa8iG8u9aqJCExcdS7BSKZpTf";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    private float skystonePos;

    //Declare imu
    private BNO055IMU imu;

    public void runOpMode() throws InterruptedException{
        //Initialize motors
        motorFrontRight = hardwareMap.dcMotor.get("FR");
        motorFrontLeft = hardwareMap.dcMotor.get("FL");
        motorBackRight = hardwareMap.dcMotor.get("BR");
        motorBackLeft = hardwareMap.dcMotor.get("BL");
        //Initialize imu
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        //Initialize intake
        leftIntake = hardwareMap.crservo.get("LI");
        rightIntake = hardwareMap.crservo.get("RI");
        //Initialize servos
        leftIntakeServo = hardwareMap.servo.get("LIservo");
        rightIntakeServo = hardwareMap.servo.get("RIservo");
        //Initialize arm
        arm = hardwareMap.dcMotor.get("arm");

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Create an IMURobot object that we will use to run the robot
        IMURobot robot = new IMURobot(motorFrontRight, motorFrontLeft, motorBackRight, motorBackLeft, imu,
                leftIntake, rightIntake, leftIntakeServo, rightIntakeServo, this);

        robot.setupRobot();//calibrate IMU, set any required parameters
        robot.resetEncoders();//reset motor encoders
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        //Testing again
        if (tfod != null) {
            tfod.activate();
        }

        telemetry.addData("Status", "Ready");
        telemetry.update();

        waitForStart(); //wait for the game to start

        telemetry.addData("Hello?", "afafaf");
        telemetry.update();

        robot.gyroDriveEncoder(0.5, 30);
        robot.gyroStrafeEncoder(0.5, 90, 50);
        sleep(500);

        int leftCount = 0, centerCount = 0, rightCount = 0;
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while(timer.seconds() < 2){
            double newPos = getSkystonePos();
            telemetry.addData("Skystone Pos", newPos);
            telemetry.update();
            if((newPos > 900 && newPos < 1500) || (newPos > -10) && (newPos < 200)){
                rightCount++;
            }else if(newPos > 200 && newPos < 500){
                centerCount++;
            }else if (newPos > 500 && newPos < 900){
                leftCount++;
            }
        }

        //ElapsedTime timer = new ElapsedTime();
        //timer.reset();

        /*telemetry.addData("Status", "Moving");
        telemetry.update();

        telemetry.addData("Status", "Hello");
        telemetry.update();*/

        /*robot.gyroStrafeEncoder(0.3, 90, 15);
        while(opModeIsActive() && timer.seconds() < 100){

            robot.gyroStrafe(0.1, 90);

            telemetry.addData("Current ticks", robot.getMotorPosition());
            telemetry.addData("Skystone pos", getSkystonePos());
            telemetry.update();
            if(getSkystonePos() > 200){
                telemetry.addData("Status", "breaking from loop");
                telemetry.update();
                sleep(1000);
                break;
            }
        }*/
        /*robot.completeStop();
        telemetry.addData("Hello", "hello??");
        telemetry.update();
        sleep(1000);*/

        //1 = left; 2 = center; 3 = right;
        int skystoneConfig;
        if(leftCount > Math.max(centerCount, rightCount)){
            skystoneConfig = 1;
        }else if(centerCount > Math.max(leftCount, rightCount)){
            skystoneConfig = 2;
        }else{
            skystoneConfig = 3;
        }
        telemetry.addData("Config", skystoneConfig);
        telemetry.update();

        switch (skystoneConfig) {
            case 1:
                /*robot.gyroTurn(180, 0.3);
                robot.releaseIntake();
                sleep(500);
                robot.intakeOn();
                robot.gyroDriveEncoder(-0.1, 30);
                sleep(250);
                robot.gyroDriveEncoder(0.5, 30);
                robot.gyroStrafeEncoder(0.5, 90, 120);
                robot.intakeReverse();
                robot.gyroDriveEncoder(0.1, 30);*/
                robot.gyroStrafeEncoder(0.5, -90, 15);
                robot.gyroTurn(175, 0.3);
                robot.releaseIntake();
                sleep(500);
                robot.intakeOn();
                robot.gyroDriveEncoder(-0.1, 40);
                sleep(250);
                robot.gyroDriveEncoder(0.75, 30);
                robot.gyroStrafeEncoder(0.75, 90, 110);
                robot.intakeReverse();
                robot.gyroDriveEncoder(0.25, 30);
                break;
            case 2:
                /*robot.gyroTurn(180, 0.3);
                //robot.intakeOn();
                robot.gyroDriveEncoder(-0.1, 30);
                sleep(250);
                robot.gyroDriveEncoder(0.5, 30);
                robot.gyroStrafeEncoder(0.5, -90, 140);*/
                robot.gyroTurn(175, 0.3);
                robot.releaseIntake();
                sleep(500);
                robot.intakeOn();
                robot.gyroDriveEncoder(-0.1, 40);
                sleep(250);
                robot.gyroDriveEncoder(0.75, 30);
                robot.gyroStrafeEncoder(0.75, 90, 130);
                robot.intakeReverse();
                robot.gyroDriveEncoder(0.25, 30);
                break;
            case 3:
                /*robot.gyroTurn(180, 0.3);
                //robot.intakeOn();
                robot.gyroDriveEncoder(-0.1, 30);
                sleep(250);
                robot.gyroDriveEncoder(0.5, 30);
                robot.gyroStrafeEncoder(0.5, -90, 160);*/
                robot.gyroStrafeEncoder(0.5, 90, 15);
                robot.gyroTurn(175, 0.3);
                robot.releaseIntake();
                sleep(500);
                robot.intakeOn();
                robot.gyroDriveEncoder(-0.1, 40);
                sleep(250);
                robot.gyroDriveEncoder(0.75, 30);
                robot.gyroStrafeEncoder(0.75, 90, 150);
                robot.intakeReverse();
                robot.gyroDriveEncoder(0.25, 30);
                break;
            default:
                robot.gyroDriveEncoder(-0.5, 120);
                break;
        }

        robot.completeStop();
        robot.gyroStrafeEncoder(0.75, -90, 20);
    }


    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.5;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    /**
     * Get skystone position
     */
    private float getSkystonePos() {
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            skystonePos = 2001;
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());

                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            recognition.getLeft(), recognition.getTop());
                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            recognition.getRight(), recognition.getBottom());
                    if(recognition.getLabel() == "Skystone"){
                        skystonePos = recognition.getLeft();
                    }
                }
                //telemetry.update();
            }
            return skystonePos;
        }else{
            return -1000;
        }
    }

    public void raiseArm() {
        while(arm.getCurrentPosition() < 500){
            arm.setPower(1);
        }
        arm.setPower(0);
    }

}