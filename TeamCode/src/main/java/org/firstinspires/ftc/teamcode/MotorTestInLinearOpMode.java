package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Motor Testing w linear op mode") //name of program in driver hub
public class MotorTestInLinearOpMode extends LinearOpMode //class MUST EXTEND OpMode
{
    DcMotor motor1;
    @Override
    public void runOpMode(){
        motor1 = hardwareMap.get(DcMotor.class,"motor1");
        /* All code for initialization goes here */

        waitForStart(); /* Tells robot to do nothing until start is hit */

        if(isStopRequested()){
            // do something BEFORE a stop
            return;
        }

        while (opModeIsActive()) { //opModeIsActive returns false only when stop is hit
            motor1.setPower(0.5);
            telemetry.addLine("Hello World!");
            telemetry.update();
        }
    }
}
