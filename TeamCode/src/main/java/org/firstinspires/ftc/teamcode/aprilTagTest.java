package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Arrays;
import java.util.List;



//calibration may be needed for cam: https://www.youtube.com/watch?v=bTcCY3DZM0k download this on windows https://www.3dflow.net/3df-zephyr-free/
@TeleOp
public class aprilTagTest extends LinearOpMode{
    double frontLeftPower = 0, backLeftPower = 0, frontRightPower = 0, backRightPower = 0;
    public void mecanumTeleopPowerCalculation(double x, double y, double rotation) {
        frontLeftPower = (y + x + rotation);
        backLeftPower = (y - x + rotation);
        frontRightPower = (y - x - rotation);
        backRightPower = (y + x - rotation);

        double motorPowers[] = {Math.abs(frontLeftPower), Math.abs(backLeftPower),
                Math.abs(frontRightPower), Math.abs(backRightPower)};

        Arrays.sort(motorPowers);
        if (motorPowers[3] > 0) {
            double velocity_factor = Math.max(Math.hypot(x, y), Math.abs(rotation));
            if (velocity_factor > 0.8) velocity_factor = 1;
            frontLeftPower = velocity_factor * frontLeftPower / motorPowers[3];
            backLeftPower = velocity_factor * backLeftPower / motorPowers[3];
            frontRightPower = velocity_factor * frontRightPower / motorPowers[3];
            backRightPower = velocity_factor * backRightPower / motorPowers[3];
        }
    }
    boolean left = true,right,center, targetFound;
    final double DESIRED_DISTANCE = 13.0; //in in
    final double MAX_AUTO_SPEED = 0.5, MAX_AUTO_TURN = 0.3, MAX_AUTO_STRAFE = 0.5;
    final double SPEED_GAIN = 0.02, STRAFE_GAIN = 0.015, TURN_GAIN = 0.01;
    AprilTagDetection desiredTag = null;
    DcMotor motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight, motorIntake, motorArm;
    @Override
    public void runOpMode() throws InterruptedException{
        motorFrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        motorFrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        motorBackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        motorBackRight = hardwareMap.get(DcMotor.class, "BackRight");
        motorIntake = hardwareMap.get(DcMotor.class, "IntakeMotor");
        motorArm = hardwareMap.get(DcMotor.class, "ArmMotor");
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);
        motorArm.setDirection(DcMotor.Direction.FORWARD);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double drive = 0, strafe = 0, turn = 0;

        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .build();
        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640,360))
                .build();
        int DESIRED_TAG_ID = -1;
        if(left){ // FOR RED
            DESIRED_TAG_ID =  4;
        }else if(center){
            DESIRED_TAG_ID = 5;
        }else if(right){
            DESIRED_TAG_ID = 6;
        }
        waitForStart();
        while(!isStopRequested() && opModeIsActive()){
            targetFound = false;
            List<AprilTagDetection> currentDetections = tagProcessor.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                        // Yes, we want to use this tag.
                        targetFound = true;
                        desiredTag = detection;
                        break;  // don't look any further.
                    } else {
                        telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                    }
                }
            }

            // Tell the driver what we see, and what to do.
            if (targetFound) {
                telemetry.addData("\n>","HOLD Left-Bumper to Drive to Target\n");
                telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
                telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
                telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);
            }

            if (targetFound) {
                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double  rangeError      = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                double  headingError    = desiredTag.ftcPose.bearing;
                double  yawError        = desiredTag.ftcPose.yaw;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            }
            telemetry.update();

            mecanumTeleopPowerCalculation(drive,strafe,turn);
            motorFrontLeft.setPower(frontLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackLeft.setPower(backLeftPower);
            motorBackRight.setPower(backRightPower);
            sleep(40);
        }
    }
}
