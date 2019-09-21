package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(name = "SkystoneDetectionTest")
public class SkystoneDetectionTest extends LinearOpMode {
    //Declare motors
    private DcMotor motorLeft;
    private DcMotor motorRight;

    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    private static final String VUFORIA_KEY = "AeXjUy3/////AAABmbHLSgwTzU5tnItQeMYIKBuJtzLC3Yjgadn90RIg4wpjZJxXPoAwCZmsm+bAMXon60mNlk3UZvSNQaabvijg0UZ+9vB/U+d8CeHrLU4FziM5JseM/zIMAdJoePSg1sli9hlC1LYIPMd6uCYwuS8QZvkhHkBisttfhafsExbOSeIP/a3sBqDAxQ7rm1SIvWxdGAgu2iUrLVMx6affe6GAtcFMRhIVWLj+m6XyRnswbtS/Sh4hNZXoBTJn4py4rhZ4iouKw6SvZvYCokuqxObpBxG8Ni0pLcbtctO5+8xtb47Y4uwkkI9t7L4IUfntsQzaMcd6eMtoPcYIhLKhiydVa8iG8u9aqJCExcdS7BSKZpTf";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    private float skystonePos;

    public void runOpMode() throws InterruptedException {
        //Initialize motors
        motorLeft = hardwareMap.dcMotor.get("left");
        motorRight = hardwareMap.dcMotor.get("right");

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
        if (tfod != null) {
            tfod.activate();
        }

        telemetry.addData("Status", "Ready");
        telemetry.update();

        waitForStart();

        while(getSkystonePos() > 200 && opModeIsActive()){
            tankDrive(0.1, 0.1);
        }
        completeStop();
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
        tfodParameters.minimumConfidence = 0.8;
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
            skystonePos = 1001;
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
                telemetry.update();
            }
            return skystonePos;
        }else{
            return 1001;
        }
    }

    private void tankDrive(double leftPower, double rightPower){
        motorLeft.setPower(leftPower);
        motorRight.setPower(-rightPower);
    }

    private void completeStop() {
        motorLeft.setPower(0);
        motorRight.setPower(0);
    }
}
