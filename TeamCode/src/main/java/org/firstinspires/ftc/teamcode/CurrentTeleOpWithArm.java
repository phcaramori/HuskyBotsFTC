package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Arrays;

@TeleOp(name="Current TeleOp Program w/ Arm")
public class CurrentTeleOpWithArm extends LinearOpMode {
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
    public int degreesToTicks(float degrees){
        int ticks = (int) Math.round(degrees*12.444);
        return ticks;
    }
    public double ticksToDegrees(int ticks){
        double degrees = (double) ticks/12.444;
        return degrees;
    }

    DcMotor motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight, motorIntake, motorArm;
    Servo servoAirplaneTrigger, servoGripper, servoWrist;
    double frontLeftPower = 0, backLeftPower = 0, frontRightPower = 0, backRightPower = 0;
    boolean intake_on = false;
    double velocity_factor;

    //TODO: Set up pos constants
    final double
            gripperPickupPos = 1, gripperScorePos = 0.4, //0 to 1
            wristPickupPos = 0, wristScorePos = 0; //0 to 1
    final int armHomePos = 0, armPickupPos = 0, armScorePos = 0; //0 to idk
    double manualArmPower = 0.0;
    boolean armManualMode = false, pixel_grab = false;

    float wristServoTarget = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        double IMPERFECT_STRAFING_MODIFIER = 1.1;
        double armPower;
        Gamepad currentGamepad = new Gamepad();
        Gamepad previousGamepad = new Gamepad();

        // ALL MOTORS:
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

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        servoAirplaneTrigger = hardwareMap.get(Servo.class, "AirplaneTriggerServo");
        servoAirplaneTrigger.setDirection(Servo.Direction.REVERSE);
        servoAirplaneTrigger.getController().pwmEnable();

        // --- ARM SETUP --- //:
        servoGripper = hardwareMap.get(Servo.class, "GripperServo");
        servoGripper.getController().pwmEnable();
        servoWrist = hardwareMap.get(Servo.class, "WristServo");
        servoWrist.getController().pwmEnable();

        motorArm = hardwareMap.get(DcMotor.class, "ArmMotor");
        motorArm.setDirection(DcMotor.Direction.REVERSE);
        motorArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArm.setPower(0.0);
        motorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // only set to run to pos later on

        telemetry.addData("Status", "Initialized");

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

            // === Intake ===
            if (gamepad1.circle && !previousGamepad.circle) {
                if (intake_on) {
                    intake_on = false;
                    motorIntake.setPower(-1);
                } else{
                    intake_on = true;
                    motorIntake.setPower(0);
                }
            }

            // === Arm ===
            manualArmPower = (gamepad1.right_trigger - gamepad1.left_trigger) / 1.33; //-0.75 - .75
            if(manualArmPower != 0.00){
                armManualMode = true;
                motorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorArm.setPower(manualArmPower);
            } else {
                armManualMode = false;
            }
            if(!armManualMode){
                if(motorArm.getMode() == DcMotor.RunMode.RUN_USING_ENCODER){ //if changing from manual to pos-based, stop arm before running to position.
                    motorArm.setPower(0);
                }
                if(gamepad1.triangle){ //score
                    if(motorArm.getCurrentPosition()==armPickupPos){
                        servoGripper.setPosition(gripperScorePos);
                    }
                    motorArm.setTargetPosition(armScorePos);
                    motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motorArm.setPower(1.0);
                }else if(gamepad1.square){ //pickup pos
                    servoGripper.setPosition(gripperPickupPos);
                    motorArm.setTargetPosition(armPickupPos);
                    motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motorArm.setPower(Math.min(Math.abs(motorArm.getCurrentPosition())/1000, 1));
                }
            }

            // === Gripper ===
            if(gamepad1.dpad_down){
                pixel_grab = true;
            }
            if(gamepad1.dpad_up){
                pixel_grab = false;
            }
            if(pixel_grab){
                servoGripper.setPosition(gripperPickupPos);
            }else{
                servoGripper.setPosition(gripperScorePos);
            }

            // === Wrist Control Based on Arm ===
            //TODO: get these constants correct
            float proportionalArmPos = motorArm.getCurrentPosition()/armScorePos; //0-100; where 0 is pickup, 1 is score.
            //wristServoTarget is = 0 for straight down, 1 for straight up; 0.5 for straight forward.
            if(proportionalArmPos <= 1){
                wristServoTarget = proportionalArmPos / 2;
            } else if(proportionalArmPos > 1){
                wristServoTarget = proportionalArmPos / 2;
            }


            // === Manual Wrist Control ===
            if(gamepad1.dpad_left){
                wristServoTarget -= 0.01;
            } else if (gamepad1.dpad_right) {
                wristServoTarget += 0.01;
            }
            servoWrist.setPosition(wristServoTarget);

            // === Airplane ===
            if (gamepad1.right_bumper){
                servoAirplaneTrigger.setPosition(1);
            } else{
                servoAirplaneTrigger.setPosition(0);
            }

            telemetry.addData("Arm Encoder Value", motorArm.getCurrentPosition());
            telemetry.addData("Gripper Servo Position", servoGripper.getPosition());
            telemetry.addData("Wrist Servo Position", servoWrist.getPosition());
            telemetry.addData("Wrist Servo Target", wristServoTarget);
            telemetry.addData("Manual Arm Power", manualArmPower);
            telemetry.addData("Manual Arm Mode (bool)", armManualMode);
            telemetry.update();
        }
    }
}
