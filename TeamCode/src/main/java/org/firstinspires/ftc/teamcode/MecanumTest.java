package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name="Mecanum Movement Testing") //name of program in driver hub
public class MecanumTest extends LinearOpMode {
    /* settings */
    double IMPERFECT_STRAFING_MODIFIER = 1.1;

    DcMotor Motor_FrontLeft, Motor_FrontRight, Motor_BackLeft, Motor_BackRight;
    @Override
    public void runOpMode() {
        Gamepad currentGamepad = new Gamepad();
        Gamepad previousGamepad = new Gamepad();

        Motor_FrontLeft = hardwareMap.get(DcMotor.class,"FrontLeft");
        Motor_FrontRight = hardwareMap.get(DcMotor.class,"FrontRight");
        Motor_BackLeft = hardwareMap.get(DcMotor.class,"BackLeft");
        Motor_BackRight = hardwareMap.get(DcMotor.class,"BackRight");

        waitForStart(); /* Tells robot to do nothing until start is hit */
        if(isStopRequested()){
            return;
        }

        while(opModeIsActive()){
            previousGamepad.copy(currentGamepad);
            currentGamepad.copy(gamepad1);

            updateMotors();
        }
    }
    public void updateMotors(){
        double y = -gamepad1.left_stick_y; //y stick is always reversed
        double x = gamepad1.left_stick_x * IMPERFECT_STRAFING_MODIFIER; //counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        Motor_FrontLeft.setPower(frontLeftPower);
        Motor_FrontRight.setPower(frontRightPower);
        Motor_BackLeft.setPower(backLeftPower);
        Motor_BackRight.setPower(backRightPower);
    }
}
