package org.firstinspires.ftc.teamcode.old;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@Autonomous(name = "TestClass")

public class TestClass extends LinearOpMode{


    public void runOpMode() throws InterruptedException{


        telemetry.addData("Status","Good");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()){

            telemetry.update();
        }
    }
}