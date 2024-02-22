package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Arrays;

@TeleOp(name="Current TeleOp Program")
public class CurrentTeleOp extends LinearOpMode {
    public void mecanumTeleopPowerCalculation(double x, double y, double rotation){
        frontLeftPower = (y + x + rotation);
        backLeftPower = (y - x + rotation);
        frontRightPower = (y - x - rotation);
        backRightPower = (y + x - rotation);

        double motorPowers[] = {Math.abs(frontLeftPower), Math.abs(backLeftPower),
                Math.abs(frontRightPower), Math.abs(backRightPower)};


        Arrays.sort(motorPowers);
        if (motorPowers[3] > 0) {
            velocity_factor = Math.max(Math.hypot(x, y), Math.abs(rotation));
            if (velocity_factor > 0.8) velocity_factor = 1;
            frontLeftPower = velocity_factor * frontLeftPower / motorPowers[3];
            backLeftPower = velocity_factor * backLeftPower / motorPowers[3];
            frontRightPower = velocity_factor * frontRightPower / motorPowers[3];
            backRightPower = velocity_factor * backRightPower / motorPowers[3];
        }


    }
    DcMotor motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight, motorIntake,
            motorArm;

    Servo servoAirplaneTrigger, servoAttackAngle, servoFinger, servoPurpleDepositor;

    double frontLeftPower = 0, backLeftPower = 0, frontRightPower = 0, backRightPower = 0;
    double a,b,c,d;
    boolean intake_on = false;
    double velocity_factor;
    @Override
    public void runOpMode() throws InterruptedException {

        double IMPERFECT_STRAFING_MODIFIER = 1.1;
        double armPower;
        Gamepad currentGamepad = new Gamepad();
        Gamepad previousGamepad = new Gamepad();

        // Motor Hardware Maps
        motorFrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        motorFrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        motorBackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        motorBackRight = hardwareMap.get(DcMotor.class, "BackRight");
        motorIntake = hardwareMap.get(DcMotor.class, "IntakeMotor");
        motorArm = hardwareMap.get(DcMotor.class, "ArmMotor");
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotor.Direction.FORWARD);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);
        motorArm.setDirection(DcMotor.Direction.REVERSE);

        //Servo Hardware Maps
        servoAirplaneTrigger = hardwareMap.get(Servo.class, "AirplaneTriggerServo");
        servoFinger = hardwareMap.get(Servo.class, "FingerServo");
        servoAttackAngle = hardwareMap.get(Servo.class, "AttackAngleServo");
        servoPurpleDepositor = hardwareMap.get(Servo.class, "PurpleDepositorServo");
        servoAirplaneTrigger.setDirection(Servo.Direction.REVERSE);
        servoAirplaneTrigger.getController().pwmEnable();
        servoFinger.getController().pwmEnable();
        servoAttackAngle.getController().pwmEnable();
        servoPurpleDepositor.getController().pwmEnable();



        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart(); /* Tells robot to do nothing until start is hit */
        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive()) {
            previousGamepad.copy(currentGamepad);
            currentGamepad.copy(gamepad1);

            double y = -(currentGamepad.left_stick_y) * (1/IMPERFECT_STRAFING_MODIFIER); //y stick is always reversed
            double x = currentGamepad.left_stick_x; //counteract imperfect strafing
            double rx = currentGamepad.right_stick_x;

            mecanumTeleopPowerCalculation(x, y, rx);


            motorFrontLeft.setPower(frontLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackLeft.setPower(backLeftPower);
            motorBackRight.setPower(backRightPower);

            Log.d("CurrentTeleOp", String.valueOf(a));

            if (intake_on) {
                motorIntake.setPower(-1);
            } else {
                motorIntake.setPower(0);
            }

            if (gamepad1.square) {
                intake_on = false;
            }
            if (gamepad1.circle) {
                intake_on = true;
            }

            if (gamepad1.right_bumper){
                servoAirplaneTrigger.setPosition(1);
            } else{
                servoAirplaneTrigger.setPosition(0);
            }

            armPower = 0;
            if (gamepad1.dpad_up) armPower = 0.5;
            else if (gamepad1.dpad_down) armPower = -0.5;
            motorArm.setPower(armPower);

        }
    }

    int encoder_ticks_to_angle(int encoder_ticks){
        // Our 180 degree angle is equivalent to XX rotations
        return 0;
    }

    int angle_to_encoder_ticks(float angle){
        return 0;
    }

}
