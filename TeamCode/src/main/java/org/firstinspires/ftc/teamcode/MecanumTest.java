package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name="Mecanum Movement Testing") //name of program in driver hub
public class MecanumTest extends LinearOpMode {
    DcMotor Motor_FrontLeft, Motor_FrontRight, Motor_BackLeft, Motor_BackRight;
    @Override
    public void runOpMode() {
        double IMPERFECT_STRAFING_MODIFIER = 1.1;
        Gamepad currentGamepad = new Gamepad();
        Gamepad previousGamepad = new Gamepad();

        Motor_FrontLeft = hardwareMap.get(DcMotor.class,"FrontLeft");
        Motor_FrontRight = hardwareMap.get(DcMotor.class,"FrontRight");
        Motor_BackLeft = hardwareMap.get(DcMotor.class,"BackLeft");
        Motor_BackRight = hardwareMap.get(DcMotor.class,"BackRight");

        Motor_FrontLeft.setDirection(DcMotor.Direction.REVERSE);
        Motor_BackLeft.setDirection(DcMotor.Direction.REVERSE);
        Motor_FrontRight.setDirection(DcMotor.Direction.FORWARD);
        Motor_BackRight.setDirection(DcMotor.Direction.REVERSE);

        Motor_FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor_BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor_FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor_BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart(); /* Tells robot to do nothing until start is hit */
        if(isStopRequested()){
            return;
        }

        while(opModeIsActive()){
            previousGamepad.copy(currentGamepad);
            currentGamepad.copy(gamepad1);

            double y = -(currentGamepad.left_stick_y); //y stick is always reversed
            double x = currentGamepad.left_stick_x * IMPERFECT_STRAFING_MODIFIER; //counteract imperfect strafing
            double rx = currentGamepad.right_stick_x;

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
}
