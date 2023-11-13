package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

//toggles a motor on and off

public class ControllerTest extends LinearOpMode {
    DcMotor Motor_FrontLeft;
    boolean toggleMotor = false;
    public void runOpMode(){
        Gamepad currentGamepad = new Gamepad();
        Gamepad previousGamepad = new Gamepad();

        Motor_FrontLeft = hardwareMap.get(DcMotor.class,"FrontLeft");

        waitForStart();

        if(isStopRequested()){
            return;
        }

        while (opModeIsActive()) {
            previousGamepad.copy(currentGamepad);
            currentGamepad.copy(gamepad1);

            if(currentGamepad.a && !previousGamepad.a){//if a is pressed and wasn't previously
                toggleMotor = !toggleMotor;
            }

            if(toggleMotor){
                Motor_FrontLeft.setPower(0.5);
            }else{
                Motor_FrontLeft.setPower(0);
            }
        }
    }
}
