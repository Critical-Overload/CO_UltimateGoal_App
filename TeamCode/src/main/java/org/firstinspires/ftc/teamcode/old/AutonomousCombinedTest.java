/* Copyright (c) 2018 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.old;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

/**
 * This 2018-2019 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the gold and silver minerals.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "AutonomousCombinedTest")
public class AutonomousCombinedTest extends LinearOpMode {

    private DcMotor motorFrontRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackRight;
    private DcMotor motorBackLeft;
    private DcMotor liftMotor;
    private DcMotor hingeMotor;
    private Servo bucketServo;
    private CRServo intakeServo;

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";


    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY = "AeXjUy3/////AAABmbHLSgwTzU5tnItQeMYIKBuJtzLC3Yjgadn90RIg4wpjZJxXPoAwCZmsm+bAMXon60mNlk3UZvSNQaabvijg0UZ+9vB/U+d8CeHrLU4FziM5JseM/zIMAdJoePSg1sli9hlC1LYIPMd6uCYwuS8QZvkhHkBisttfhafsExbOSeIP/a3sBqDAxQ7rm1SIvWxdGAgu2iUrLVMx6affe6GAtcFMRhIVWLj+m6XyRnswbtS/Sh4hNZXoBTJn4py4rhZ4iouKw6SvZvYCokuqxObpBxG8Ni0pLcbtctO5+8xtb47Y4uwkkI9t7L4IUfntsQzaMcd6eMtoPcYIhLKhiydVa8iG8u9aqJCExcdS7BSKZpTf";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;
    @Override
    public void runOpMode() throws InterruptedException{
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }

            motorFrontRight = hardwareMap.dcMotor.get("FR");
            motorFrontLeft = hardwareMap.dcMotor.get("FL");
            motorBackRight = hardwareMap.dcMotor.get("BR");
            motorBackLeft = hardwareMap.dcMotor.get("BL");
            liftMotor = hardwareMap.dcMotor.get("lift");
            hingeMotor = hardwareMap.dcMotor.get("hingeMotor");
            bucketServo = hardwareMap.servo.get("bucketServo");
            intakeServo = hardwareMap.crservo.get("intakeServo");

            motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
            motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);

            hingeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            hingeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        /*
        DriveControl is our driving class.
         */

            //DrivingControlTwo robot = new DrivingControlTwo(motorFrontRight, motorFrontLeft, motorBackRight, motorBackLeft,hingeMotor,bucketServo,intakeServo);

            while (opModeIsActive()) {


                moveArm();

                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() == 2) {
                            int goldMineralX = -1;
                            int silverMineral1X = -1;
                            int silverMineral2X = -1;
                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    goldMineralX = (int) recognition.getLeft();
                                } else if (silverMineral1X == -1) {
                                    silverMineral1X = (int) recognition.getLeft();
                                } else {
                                    silverMineral2X = (int) recognition.getLeft();
                                }
                            }


                            if (goldMineralX != -1 || silverMineral1X != -1 || silverMineral2X != -1) {
                                if (silverMineral1X < silverMineral2X) {
                                    telemetry.addData("Gold Mineral Position", "Left");
                                    //robot.fourWheelTurn(-1,70);
                                    //robot.drive(-1,35);
                                    //robot.fourWheelTurn(1,30);
                                    //robot.drive(-1,35);
                                    moveArm();
                                    intakeServo.setPower(1);
                                    sleep(3000);
                                    intakeServo.setPower(0);
                                    //Hydralax
                                    //robot.drive(1,35);
                                    //robot.fourWheelTurn(-1,30);
                                    //robot.drive(1,35);


                                } else if (goldMineralX > silverMineral1X) {
                                    telemetry.addData("Gold Mineral Position", "Right");
                                    //robot.fourWheelTurn(1,10);
                                    //robot.drive(-1,35);
                                    //robot.fourWheelTurn(-1,60);
                                    //robot.drive(-1,55);
                                    moveArm();
                                    intakeServo.setPower(1);
                                    sleep(3000);
                                    intakeServo.setPower(0);

                                    //Hydralax Code
                                    //robot.drive(1,55);
                                    //robot.fourWheelTurn(1,30);
                                    //robot.drive(1,35);




                                } else if(goldMineralX < silverMineral1X){
                                    telemetry.addData("Gold Mineral Position", "Center");
                                    //robot.fourWheelTurn(-1,15);
                                    //robot.drive(-1,55);
                                    moveArm();
                                    intakeServo.setPower(1);
                                    sleep(3000);
                                    intakeServo.setPower(0);

                                    //Hydralax
                                    //robot.drive(1,55);


                                }
                            }
                        }
                        telemetry.update();
                    }
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }
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
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    public void moveArm() throws InterruptedException {
        double bp = 1.0;
        bucketServo.scaleRange(0.1,1);
        double maxH=1;
        int ticksCycle = 1440;

        hingeMotor.setTargetPosition(3*ticksCycle);
        hingeMotor.setPower(maxH);
        while(hingeMotor.isBusy()){
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
                if(hpercent < 0.03){ //percent of max power deemed minimum to run
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


            //automatically set bucket servo to optimized position
            double bpercent = 0;
            if(hp >= 0 && hp < (1.2*ticksCycle)){
                //set between 1 and .4
                bpercent = hp/(1.2*ticksCycle);
                bp = 1-(bpercent*(1-.5));
            }
            else if(hp >= (1.2*ticksCycle) && hp <= (3*ticksCycle)){
                //set between .4 and .0
                bpercent = (hp - 1.2*ticksCycle)/(3*ticksCycle-1.2*ticksCycle);
                bp = .4-(bpercent*(.5-.0));
            }

            bucketServo.setPosition(bp);

        }
        hingeMotor.setPower(0);
        intakeServo.setPower(1);
        Thread.sleep(3000);
        intakeServo.setPower(0);
        Thread.sleep(100);
        hingeMotor.setTargetPosition(0);
        hingeMotor.setPower(maxH);
        while(hingeMotor.isBusy()){
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
                if(hpercent < 0.03){ //percent of max power deemed minimum to run
                    hpercent = 0.001;
                }
                telemetry.addData("hpercent", hpercent);
                double power = maxH * hpercent;

                hingeMotor.setPower(maxH);
                
            }
            else{
                hingeMotor.setPower(maxH);
            }
            telemetry.addData("hingePower", hingeMotor.getPower());


            //automatically set bucket servo to optimized position
            double bpercent = 0;
            if(hp >= 0 && hp < (1.2*ticksCycle)){
                //set between 1 and .4
                bpercent = hp/(1.2*ticksCycle);
                bp = 1-(bpercent*(1-.5));
            }
            else if(hp >= (1.2*ticksCycle) && hp <= (3*ticksCycle)){
                //set between .4 and .0
                bpercent = (hp - 1.2*ticksCycle)/(3*ticksCycle-1.2*ticksCycle);
                bp = .4-(bpercent*(.5-.0));
            }

            bucketServo.setPosition(bp);

        }
        hingeMotor.setPower(0);
    }

}
