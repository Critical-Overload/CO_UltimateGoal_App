/*
 * Copyright (c) 2019 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;


@Autonomous(name = "SkystoneAndFoundationDetectionWebcam")
public class SkystoneAndFoundationDetectionWebcam extends LinearOpMode
{
    double hue;
    double huetwo;
    OpenCvCamera webcam;
    MainPipeline mainPipeline;
    double threshold = 150;
    double sensitivity;
    String side = "";
    double sideM;
    double sideS;
    double thing;
    boolean onoff;

    private DcMotor motorFrontRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackRight;
    private DcMotor motorBackLeft;
    private CRServo leftIntake;
    private CRServo rightIntake;
    private Servo leftIntakeServo;
    private Servo rightIntakeServo;
    private Servo flimsy;

    //Declare imu
    private BNO055IMU imu;

    @Override
    public void runOpMode() throws InterruptedException
    {
        motorFrontRight = hardwareMap.dcMotor.get("FR");
        motorFrontLeft = hardwareMap.dcMotor.get("FL");
        motorBackRight = hardwareMap.dcMotor.get("BR");
        motorBackLeft = hardwareMap.dcMotor.get("BL");
        leftIntake = hardwareMap.crservo.get("LI");
        rightIntake = hardwareMap.crservo.get("RI");
        leftIntakeServo = hardwareMap.servo.get("LIrelease");
        rightIntakeServo = hardwareMap.servo.get("RIrelease");
        flimsy = hardwareMap.servo.get("flimsy");

        //Initialize imu
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        //Reverse requred motors

        //Set zero power behaviors to brake
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Create an IMURobot object that we will use to run the robot
        IMURobot robot = new IMURobot(motorFrontRight, motorFrontLeft, motorBackRight, motorBackLeft, imu, leftIntake, rightIntake, leftIntakeServo, rightIntakeServo, flimsy, this);
        robot.setupRobot();//calibrate IMU, set any required parameters

        /*
         * Instantiate an OpenCvCamera object for the camera we'll be using.
         * In this sample, we're using the phone's internal camera. We pass it a
         * CameraDirection enum indicating whether to use the front or back facing
         * camera, as well as the view that we wish to use for camera monitor (on
         * the RC phone). If no camera monitor is desired, use the alternate
         * single-parameter constructor instead (commented out below)
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = new OpenCvWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View
        //phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK);

        webcam.openCameraDevice();

        mainPipeline = new MainPipeline();

        webcam.setPipeline(mainPipeline);


        webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);


        waitForStart();


        telemetry.addData("Center Point", mainPipeline.bcenterx + "," + mainPipeline.bcentery);
        //Input Upright Mid Point: 240,320
        //Input Sideways Mid Point: 320,240
        double inputCenterX = 320;
        double accuracy = 30;
        thing = 1;

        if (mainPipeline.bcenterx > inputCenterX + accuracy){
            side = "Right";
            sideM = 1;
        }else if (mainPipeline.bcenterx < inputCenterX - accuracy){
            side = "Left";
            sideM = -1;
        }
        else{
            side = "In the Center";
            sideM = 0;

        }
        if (mainPipeline.scenterx > inputCenterX + accuracy){
            side = "Right";
            sideS = 1;
        }else if (mainPipeline.scenterx < inputCenterX - accuracy){
            side = "Left";
            sideS = -1;
        }
        else{
            side = "In the Center";
            sideS = 0;

        }
        // 0 = foundation
        // 1 = skystone
        robot.flimsyUp();
        robot.gyroStrafeEncoder(1,90,30);
        robot.gyroDriveEncoder(-0.7,15);

        while (thing == 1 && sideS != 0){

            robot.tankDrive(-0.4,-0.7);
            robot.resetAngle();

            //Input Upright Mid Point: 240,320
            //Input Sideways Mid Point: 320,240
            if (mainPipeline.scenterx > inputCenterX + accuracy){
                side = "Right";
                sideS = 1;

            }else if (mainPipeline.scenterx < inputCenterX - accuracy){
                side = "Left";
                sideS = -1;

            }
            else{
                side = "In the Center";
                sideS = 0;

            }
            telemetry.update();

        }
        robot.completeStop();
        robot.gyroDriveEncoder(.4,10);
        robot.gyroStrafeEncoder(.4,90,42);
        robot.flimsyDown();
        sleep(500);
        robot.gyroStrafeEncoder(1,-90,20);
        robot.gyroDriveEncoder(1,60);
        flimsy.setPosition(0.8);
        robot.gyroDriveEncoder(-1,40);

        //New Changes
        robot.gyroDriveEncoder(-0.7,15);

        while (thing == 1 && sideS != 0){

            robot.tankDrive(-0.7,-0.7);
            robot.resetAngle();

            //Input Upright Mid Point: 240,320
            //Input Sideways Mid Point: 320,240
            if (mainPipeline.scenterx > inputCenterX + accuracy){
                side = "Right";
                sideS = 1;

            }else if (mainPipeline.scenterx < inputCenterX - accuracy){
                side = "Left";
                sideS = -1;

            }
            else{
                side = "In the Center";
                sideS = 0;

            }
            telemetry.update();

        }








    }

    class MainPipeline extends OpenCvPipeline
    {
        List<MatOfPoint> bcontours = new ArrayList<>();
        List<MatOfPoint> scontours = new ArrayList<>();
        List<MatOfPoint> ycontours = new ArrayList<>();


        int bcenterx;
        int bcentery;
        int scenterx;
        int scentery;
        Mat hsvImage = new Mat();
        Mat buildplate = new Mat();
        Mat blurImg = new Mat();
        Mat cannyOutput = new Mat();
        Mat output = new Mat();
        Mat yellow = new Mat();
        Scalar myColor = new Scalar(0,255,255);
        Mat grey = new Mat();
        Mat greyImg = new Mat();

        @Override
        public Mat processFrame(Mat input) {
            input.copyTo(output);
            Mat mask = new Mat(input.rows(), input.cols(), CvType.CV_8U, Scalar.all(0));
            Mat cropped = new Mat(input.size(),input.type(),myColor);
            ycontours.clear();

            bcontours.clear();
            scontours.clear();

            Imgproc.Canny(input, cannyOutput, threshold, threshold * 2);

            //yellow = 60
            //Blue = 240
            //red = 0 or 360
            hue = 360;
            sensitivity = 20;
            huetwo = 50;


            Imgproc.GaussianBlur(input, blurImg, new Size(5, 5), 0);

            //converting blured image from BGR to HSV
            Imgproc.cvtColor(blurImg, hsvImage, Imgproc.COLOR_RGB2HSV);

            Core.inRange(hsvImage, new Scalar((hue / 2) - sensitivity, 100, 50), new Scalar((hue / 2) + sensitivity, 255, 255), buildplate);


            Imgproc.findContours(buildplate, bcontours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            Core.inRange(hsvImage, new Scalar((huetwo / 2) - sensitivity, 100, 50), new Scalar((huetwo / 2) + sensitivity, 255, 255), yellow);
            Imgproc.findContours(yellow, ycontours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

            if (ycontours.size() > 0){

                double bmaxVal = 0;
                int bmaxValIdx = 0;
                for (int contourIdx = 0; contourIdx < ycontours.size(); contourIdx++) {
                    double contourArea = Imgproc.contourArea(ycontours.get(contourIdx));
                    if (bmaxVal < contourArea) {
                        bmaxVal = contourArea;
                        bmaxValIdx = contourIdx;
                    }
                }
                        Rect ylargestRect = Imgproc.boundingRect(ycontours.get(bmaxValIdx ));
                        Imgproc.rectangle(mask, new Point(0, ylargestRect.y-5), new Point(640, ylargestRect.y + ylargestRect.height), new Scalar(255, 255, 255), -1, 8, 0);
                        Imgproc.line(output, new Point(0,ylargestRect.y-5), new Point(640, ylargestRect.y-5), new Scalar(50,50,50));
                        Imgproc.rectangle(output, new Point(0, ylargestRect.y), new Point(640, ylargestRect.y + ylargestRect.height), new Scalar(255, 0, 0), 1, 8, 0);

                        input.copyTo(cropped, mask);
                        cropped.copyTo(input);

                Imgproc.cvtColor(input,grey, Imgproc.COLOR_RGB2GRAY);
                Imgproc.threshold(grey, greyImg,20,255,Imgproc.THRESH_BINARY_INV);
                Imgproc.findContours(greyImg, scontours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            }


            if (bcontours.size()>0)
            {

                //For each contour found

                double bmaxVal = 0;
                int bmaxValIdx = 0;
                for (int contourIdx = 0; contourIdx < bcontours.size(); contourIdx++) {
                    double contourArea = Imgproc.contourArea(bcontours.get(contourIdx));
                    if (bmaxVal < contourArea) {
                        bmaxVal = contourArea;
                        bmaxValIdx = contourIdx;
                    }
                }

                Imgproc.drawContours(output, bcontours, bmaxValIdx, new Scalar(0, 255, 0), 3);

                Rect blargestRect = Imgproc.boundingRect(bcontours.get(bmaxValIdx));
                Imgproc.rectangle(output, blargestRect.tl(), blargestRect.br(), new Scalar(0, 255, 0), 1, 8, 0);
                bcenterx = (blargestRect.x +blargestRect.x + blargestRect.width)/2;
                bcentery = (blargestRect.y + blargestRect.y + blargestRect.height)/2;



            }
            if(scontours.size()>0)
            {

                double smaxVal = 0;
                int smaxValIdx = 0;
                for (int contourIdx = 0; contourIdx < scontours.size(); contourIdx++) {
                    double contourArea = Imgproc.contourArea(scontours.get(contourIdx));
                    if (smaxVal < contourArea) {
                        smaxVal = contourArea;
                        smaxValIdx = contourIdx;
                    }
                }

                Imgproc.drawContours(output, scontours, smaxValIdx, new Scalar(0, 255, 0), 3);

                Rect slargestRect = Imgproc.boundingRect(scontours.get(smaxValIdx));
                Imgproc.rectangle(output, slargestRect.tl(), slargestRect.br(), new Scalar(100, 255, 100), 1, 8, 0);
                scenterx = (slargestRect.x +slargestRect.x + slargestRect.width)/2;
                scentery = (slargestRect.y + slargestRect.y + slargestRect.height)/2;



            }
            if (bcontours.size()>0 && scontours.size()>0){

                    double smaxVal = 0;
                    int smaxValIdx = 0;
                    for (int contourIdx = 0; contourIdx < scontours.size(); contourIdx++) {
                        double contourArea = Imgproc.contourArea(scontours.get(contourIdx));
                        if (smaxVal < contourArea) {
                            smaxVal = contourArea;
                            smaxValIdx = contourIdx;
                        }
                    }

                    Imgproc.drawContours(output, scontours, smaxValIdx, new Scalar(0, 255, 0), 3);

                    Rect slargestRect = Imgproc.boundingRect(scontours.get(smaxValIdx));
                    Imgproc.rectangle(output, slargestRect.tl(), slargestRect.br(), new Scalar(100, 255, 100), 1, 8, 0);
                    scenterx = (slargestRect.x +slargestRect.x + slargestRect.width)/2;
                    scentery = (slargestRect.y + slargestRect.y + slargestRect.height)/2;

                    double bmaxVal = 0;
                    int bmaxValIdx = 0;
                    for (int contourIdx = 0; contourIdx < bcontours.size(); contourIdx++) {
                        double contourArea = Imgproc.contourArea(bcontours.get(contourIdx));
                        if (bmaxVal < contourArea) {
                            bmaxVal = contourArea;
                            bmaxValIdx = contourIdx;
                        }
                    }

                    Imgproc.drawContours(output, bcontours, bmaxValIdx, new Scalar(0, 255, 0), 3);

                    Rect blargestRect = Imgproc.boundingRect(bcontours.get(bmaxValIdx));
                    Imgproc.rectangle(output, blargestRect.tl(), blargestRect.br(), new Scalar(0, 255, 0), 1, 8, 0);
                    bcenterx = (blargestRect.x +blargestRect.x + blargestRect.width)/2;
                    bcentery = (blargestRect.y + blargestRect.y + blargestRect.height)/2;





            }


            return output;
        }

    }
}