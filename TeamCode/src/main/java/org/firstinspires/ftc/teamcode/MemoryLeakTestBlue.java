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

import android.renderscript.ScriptGroup;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.MatOfPoint2f;
import org.opencv.core.CvType;
import org.opencv.core.Core;
import org.opencv.core.Size;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;


@TeleOp (name = "MemoryLeakTestBlue")
public class MemoryLeakTestBlue extends LinearOpMode
{

    double hue;
    double huetwo;
    double threshold = 150;
    double sensitivity;
    OpenCvCamera phoneCam;

    MainPipeline mainPipeline;
    String side = "";


    @Override
    public void runOpMode()
    {
        /*
         * Instantiate an OpenCvCamera object for the camera we'll be using.
         * In this sample, we're using the phone's internal camera. We pass it a
         * CameraDirection enum indicating whether to use the front or back facing
         * camera, as well as the view that we wish to use for camera monitor (on
         * the RC phone). If no camera monitor is desired, use the alternate
         * single-parameter constructor instead (commented out below)
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);


        // OR...  Do Not Activate the Camera Monitor View
        //phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK);

        /*
         * Open the connection to the camera device
         */
        phoneCam.openCameraDevice();

        /*
         * Specify the image processing pipeline we wish to invoke upon receipt
         * of a frame from the camera. Note that switching pipelines on-the-fly
         * (while a streaming session is in flight) *IS* supported.
         */
        mainPipeline = new MainPipeline();

        phoneCam.setPipeline(mainPipeline);

        /*
         * Tell the camera to start streaming images to us! Note that you must make sure
         * the resolution you specify is supported by the camera. If it is not, an exception
         * will be thrown.
         *
         * Also, we specify the rotation that the camera is used in. This is so that the image
         * from the camera sensor can be rotated such that it is always displayed with the image upright.
         * For a front facing camera, rotation is defined assuming the user is looking at the screen.
         * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
         * away from the user.
         */
        phoneCam.startStreaming(640, 480, OpenCvCameraRotation.SIDEWAYS_LEFT);

        /*
         * Wait for the user to press start on the Driver Station
         */
        waitForStart();

        while (opModeIsActive())
        {
            /*
             * Send some stats to the telemetry
             */

            telemetry.addData("Center Point", mainPipeline.bcenterx + "," + mainPipeline.bcentery);
            //Input Upright Mid Point: 240,320
            //Input Sideways Mid Point: 320,240
            double inputCenterX = 240;
            double accuracy = 30;
            if (mainPipeline.bcenterx > inputCenterX + accuracy){
                side = "Right";
            }else if (mainPipeline.bcenterx < inputCenterX - accuracy){
                side = "Left";
            }
            else{
                side = "In the Center";
            }
            telemetry.addData("Side to Move:",side );

            telemetry.addData("Frame Count", phoneCam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", phoneCam.getFps()));
            telemetry.addData("Total frame time ms", phoneCam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", phoneCam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", phoneCam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", phoneCam.getCurrentPipelineMaxFps());

            telemetry.update();
        }
    }

    /*
     * An example image processing pipeline to be run upon receipt of each frame from the camera.
     * Note that the processFrame() method is called serially from the frame worker thread -
     * that is, a new camera frame will not come in while you're still processing a previous one.
     * In other words, the processFrame() method will never be called multiple times simultaneously.
     *
     * However, the rendering of your processed image to the viewport is done in parallel to the
     * frame worker thread. That is, the amount of time it takes to render the image to the
     * viewport does NOT impact the amount of frames per second that your pipeline can process.
     *
     * IMPORTANT NOTE: this pipeline is NOT invoked on your OpMode thread. It is invoked on the
     * frame worker thread. This should not be a problem in the vast majority of cases. However,
     * if you're doing something weird where you do need it synchronized with your OpMode thread,
     * then you will need to account for that accordingly.
     */
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
            hue = 240;
            sensitivity = 20;
            huetwo = 50;


            //blur image
            Imgproc.GaussianBlur(input, blurImg, new Size(5, 5), 0);

            //converting blurred image from BGR to HSV
            Imgproc.cvtColor(blurImg, hsvImage, Imgproc.COLOR_RGB2HSV);

            //find yellow contours
            Core.inRange(hsvImage, new Scalar((huetwo / 2) - sensitivity, 100, 50), new Scalar((huetwo / 2) + sensitivity, 255, 255), yellow);
            Imgproc.findContours(yellow, ycontours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

            Core.inRange(hsvImage, new Scalar((hue / 2) - sensitivity, 100, 50), new Scalar((hue / 2) + sensitivity, 255, 255), buildplate);
            Imgproc.findContours(buildplate, bcontours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);



            if (ycontours.size() > 0){

                //Find the largest yellow contour
                double bmaxVal = 0;
                int bmaxValIdx = 0;
                for (int contourIdx = 0; contourIdx < ycontours.size(); contourIdx++) {
                    double contourArea = Imgproc.contourArea(ycontours.get(contourIdx));
                    if (bmaxVal < contourArea) {
                        bmaxVal = contourArea;
                        bmaxValIdx = contourIdx;
                    }
                }
                //Find the bounding box of the largest yellow contour
                Rect ylargestRect = Imgproc.boundingRect(ycontours.get(bmaxValIdx ));
                Imgproc.rectangle(mask, new Point(0, ylargestRect.y), new Point(640, ylargestRect.y + ylargestRect.height), new Scalar(255, 255, 255), -1, 8, 0);

                Imgproc.line(output, new Point(0,ylargestRect.y), new Point(640, ylargestRect.y), new Scalar(50,50,50));
                Imgproc.rectangle(output, new Point(0, ylargestRect.y), new Point(640, ylargestRect.y + ylargestRect.height), new Scalar(255, 0, 0), 1, 8, 0);

                //create mask
                input.copyTo(cropped, mask);
                cropped.copyTo(input);

                //find black contours
                Imgproc.cvtColor(input,grey, Imgproc.COLOR_RGB2GRAY);
                Imgproc.threshold(grey, greyImg,30,255,Imgproc.THRESH_BINARY_INV);
                Imgproc.findContours(greyImg, scontours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            }

            if(scontours.size()>0)
            {

                //find largest black contour
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

                // find bounding box and center point for largest black contour
                Rect slargestRect = Imgproc.boundingRect(scontours.get(smaxValIdx));
                Imgproc.rectangle(output, slargestRect.tl(), slargestRect.br(), new Scalar(100, 255, 100), 1, 8, 0);
                scenterx = (slargestRect.x +slargestRect.x + slargestRect.width)/2;
                scentery = (slargestRect.y + slargestRect.y + slargestRect.height)/2;

            }


            return output;
        }
    }


}