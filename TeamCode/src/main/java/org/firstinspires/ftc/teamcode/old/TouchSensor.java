package org.firstinspires.ftc.teamcode.old;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@Autonomous(name = "TouchTest")

public class TouchSensor extends LinearOpMode{

    private DigitalChannel touchSensor;
    
    public void runOpMode() throws InterruptedException{
        
        touchSensor = hardwareMap.digitalChannel.get("touchSensor");
        
        telemetry.addData("Status","Good");
        telemetry.update();
        
        waitForStart();
        
        while(opModeIsActive()){
            
            if(touchSensor.getState() == false){
                telemetry.addData("Touch Sensor","Pressed");
            }else{
                telemetry.addData("Touch Sensor","Not pressed");
            }
            
            telemetry.update();
        }
    }
}