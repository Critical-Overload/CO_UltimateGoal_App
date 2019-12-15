package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.opencv.core.Mat;

//Created October 20, 2018 by Jonathan

/* 
Uses Java built in trig functions to calculate motor powers
gamepad 1 right stick controls direction
gamepad 1 left stick controls rotation
*/

@TeleOp(name = "ThreadTest")
public class ThreadTest extends LinearOpMode implements Runnable
{
    double x;
    @Override

    public void run()
    {
        try
        {

            telemetry.addData("Thread", "Thread " + x);
            x = x+1;
            telemetry.update();

        }
        catch (Exception e)
        {
        }
    }

    public void runOpMode () throws InterruptedException
    {

        waitForStart();

        while(opModeIsActive())
        {
            run();
            telemetry.update();
        }
    }

    class Multithread
    {
        public void main(String[] args)
        {
            int n = 8; // Number of threads
            for (int i=0; i<8; i++)
            {
                Thread object = new Thread(new ThreadTest());
                object.start();
            }
        }
    }
}

