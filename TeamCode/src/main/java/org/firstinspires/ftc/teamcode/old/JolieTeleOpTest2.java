package org.firstinspires.ftc.teamcode.old;
// Imports are added automatically when you type in new code

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

//@TeleOp means that this code will appear on the TeleOp dropdown menu when you open the
//driver station. "Jolie Test" is the name it will display.
@TeleOp(name = "Jolie TeleOp Test2")
// If you didn't want this to appear on the list, you add @Disabled

// You put extends LinearOpMode at end because it helps us gain access to commands in the FTC SDK
//for Linear OpModes
public class JolieTeleOpTest2 extends LinearOpMode
{
    //This declares the motors (like defining variables)
    private DcMotor motorFrontRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackRight;
    private DcMotor motorBackLeft;

    // The runOpMode is required when writing a Linear OpMode
    @Override
    public void runOpMode() throws InterruptedException
    {
        // This code is run when you click init.

        //This links the variables to the actual motor.
        motorBackLeft = hardwareMap.dcMotor.get("1");
        motorBackRight = hardwareMap.dcMotor.get("0");
        motorFrontLeft = hardwareMap.dcMotor.get("3");
        motorFrontRight = hardwareMap.dcMotor.get("2");


        //We reverse a motor from each pair so that the robot doesn't spin but actually moves.
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);

        // waitForStart is required so that the code doesn't start running as you click init.
        //Instead, it waits till you click start
        waitForStart();

        //This code is run after you click Start.
        //This While loop will run until the stop button is pressed.

        while(opModeIsActive())
        {
            //This makes it so that moving the right stick forward or backward moves the robot
            // forward (motors 1 and 3 turn), and moving the stick to the left or right makes the
            //robot move sideways (motors 2 and 0 turn). Putting the stick diagonally forward or
            // backwards makes the robot move diagonally (all motors turn).
            //There is a negative sign in front of the gamepad1 because moving the joystick forward
            // actually returns a value of -1 instead of 1, so the robot would actually move
            // backwards. The negative sign fixes that.
            motorFrontLeft.setPower(-gamepad1.right_stick_y);
            motorBackLeft.setPower(-gamepad1.right_stick_y);
            motorBackRight.setPower(gamepad1.right_stick_x);
            motorFrontRight.setPower(gamepad1.right_stick_x);
            

            if (gamepad1.left_stick_x == 1) {
                motorBackRight.setDirection(DcMotor.Direction.FORWARD);
                motorBackLeft.setDirection(DcMotor.Direction.FORWARD);
                motorBackLeft.setPower(-1);
                motorFrontLeft.setPower(-1);
                motorBackRight.setPower(-1);
                motorFrontRight.setPower(-1);

            } else if (gamepad1.left_stick_x == -1) {
                motorBackRight.setDirection(DcMotor.Direction.FORWARD);
                motorBackLeft.setDirection(DcMotor.Direction.FORWARD);
                motorBackLeft.setPower(1);
                motorFrontLeft.setPower(1);
                motorBackRight.setPower(1);
                motorFrontRight.setPower(1);
            }else{
                motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
                motorFrontLeft.setDirection(DcMotor.Direction.FORWARD);
                motorFrontRight.setDirection(DcMotor.Direction.FORWARD);
                motorBackRight.setDirection(DcMotor.Direction.REVERSE);
            }

                //test
                // This idle command is like a buffer, and gives the hardware a chance to catch
                //up to the code.
                idle();
        }

    }
}
