package org.firstinspires.ftc.teamcode.autonomi;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
@Autonomous(name="A4 (Blue Team Close to Board)")
public class A4 extends LinearOpMode {

    DcMotor motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight, motorIntake,
            motorArm;

    Servo servoAirplaneTrigger, servoWrist, servoGripper, servoPurpleDepositor;
    int teamPropPlacement = 2;
    double cX = 0;
    double cY = 0;

    private OpenCvCamera controlHubCam;  // Use OpenCvCamera class from FTC SDK
    private static final int CAMERA_WIDTH = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution

    final double
            gripperClosedPos = 1, gripperOpenedPos = 0.5, //0 to 1
            wristPickupPos = 0, wristScorePos = 1; //0 to 1
    final int armPickupPos = 0, armScorePos = degreesToTicks(150); //0 to idk
    double wristServoTarget = 0;
    float proportionalArmPos = 0;

    public int TeamPropPositionCalculator(){
//        telemetry.addData("cX", cX);
        if(cX < 200){
//            telemetry.addData("Team Prop Location: ", "Left");
//            telemetry.update();
            return 0; //Left
        } else if(cX < 400){
//            telemetry.addData("Team Prop Location: ", "Center");
//            telemetry.update();
            return 1; //center
        } else{
//            telemetry.addData("Team Prop Location: ", "Right");
//            telemetry.update();
            return 2; //right
        }
    }


    @Override
    public void runOpMode() throws InterruptedException {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
//        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);

        motorIntake = hardwareMap.get(DcMotor.class, "IntakeMotor");
        motorArm = hardwareMap.get(DcMotor.class, "ArmMotor");
        motorArm.setDirection(DcMotor.Direction.REVERSE);

        //Servo Hardware Maps
        servoAirplaneTrigger = hardwareMap.get(Servo.class, "AirplaneTriggerServo");
        servoPurpleDepositor = hardwareMap.get(Servo.class, "PurpleDepositorServo");
        servoPurpleDepositor.getController().pwmEnable();
        servoPurpleDepositor.setDirection(Servo.Direction.REVERSE);
        servoAirplaneTrigger.setDirection(Servo.Direction.REVERSE);
        servoAirplaneTrigger.getController().pwmEnable();
        servoPurpleDepositor.getController().pwmEnable();

        //Arm Setup
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

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d starting_position = new Pose2d(56.8, -11.6, 0);
        Trajectory leftProp = drive.trajectoryBuilder(starting_position)
                .splineToLinearHeading(new Pose2d(46, -21.4, Math.PI/2), 0,
                        SampleMecanumDrive.getVelocityConstraint(
                                DriveConstants.MAX_VEL,
                                DriveConstants.MAX_ANG_VEL,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * 0.5))
                .build();
        Trajectory leftDisengageStrafe = drive.trajectoryBuilder(leftProp.end())
                .strafeLeft(10).build();
        Trajectory leftBackboard = drive.trajectoryBuilder(leftDisengageStrafe.end())
                .splineToLinearHeading(new Pose2d(36.2, -46, Math.PI/2), leftDisengageStrafe.end().getHeading())
                .build();
        Trajectory centerProp = drive.trajectoryBuilder(starting_position)
                .splineToLinearHeading(new Pose2d(26.3, -24.1, Math.PI), starting_position.getHeading())
                .build();
        Trajectory centerDisengageStrafe = drive.trajectoryBuilder(centerProp.end())
                .strafeLeft(10).build();
        Trajectory centerBackboard = drive.trajectoryBuilder(centerDisengageStrafe.end())
                .splineToLinearHeading(new Pose2d(36.2, -46, Math.PI/2), centerDisengageStrafe.end().getHeading())
                .build();

        Trajectory RightPropXPositioning = drive.trajectoryBuilder(starting_position)
                .lineToLinearHeading(new Pose2d(36.7, starting_position.getY(), 0))
                .build();
        Trajectory rightProp = drive.trajectoryBuilder(RightPropXPositioning.end())
                .strafeRight(3).build();
        Trajectory rightDisengageStrafe = drive.trajectoryBuilder(leftProp.end())
                .strafeLeft(10).build();
        Trajectory rightBackboard = drive.trajectoryBuilder(leftDisengageStrafe.end())
                .splineToLinearHeading(new Pose2d(36.2, -46, Math.PI/2), leftDisengageStrafe.end().getHeading())
                .build();
        Trajectory tuckAway = drive.trajectoryBuilder(rightBackboard.end())
                .strafeRight(20).build();
        initOpenCVRed();
        while(opModeInInit()){
            teamPropPlacement = TeamPropPositionCalculator();
        }
        waitForStart();
        servoPurpleDepositor.setPosition(0);
        drive.setPoseEstimate(starting_position);
        if(teamPropPlacement == 0){
            telemetry.addData("TeamPropPosition: ", "Left");
            telemetry.update();
            drive.followTrajectory(leftProp);
            servoPurpleDepositor.setPosition(0.7);
            sleep(1000);
            drive.followTrajectory(leftDisengageStrafe);
            drive.followTrajectory(leftBackboard);
        } else if(teamPropPlacement == 1){
            telemetry.addData("TeamPropPosition: ", "Center");
            telemetry.update();
            drive.followTrajectory(centerProp);
            servoPurpleDepositor.setPosition(0.7);
            sleep(1000);
            drive.followTrajectory(centerDisengageStrafe);
            drive.followTrajectory(centerBackboard);
        } else{
            telemetry.addData("TeamPropPosition: ", "Right");
            telemetry.update();
            drive.followTrajectory(RightPropXPositioning);
            drive.followTrajectory(rightProp);
            servoPurpleDepositor.setPosition(0.7);
            sleep(1000);
            drive.followTrajectory(rightDisengageStrafe);
            drive.followTrajectory(rightBackboard);
        }

        servoGripper.setPosition(gripperClosedPos);
        motorArm.setTargetPosition(armScorePos+degreesToTicks(15));
        motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorArm.setPower(0.25);

        while(opModeIsActive()){
            proportionalArmPos = (float) motorArm.getCurrentPosition() / (float) armScorePos; //0-1; where 0 is pickup, 1 is score.
            //wristServoTarget is = 0 for straight down, 1 for straight up; 0.5 for straight forward.
            if(proportionalArmPos <= 0.15){
                int constant = 10;
                wristServoTarget = Math.pow(constant*(0.15 - proportionalArmPos), 1.75)/constant; //quadratic, ends at ~.15
            }else if(proportionalArmPos <= 0.25){
                wristServoTarget = 0;
            } else if(proportionalArmPos <= 1.00) {
                wristServoTarget = (proportionalArmPos - 0.25)/0.75 * wristScorePos; // subtraction and division eliminate a potential jump discontinuity
            } else if(proportionalArmPos > 1.00){
                wristServoTarget = wristScorePos - 1*(proportionalArmPos - 1);
            }
            servoWrist.setPosition(wristServoTarget);
            if(motorArm.getCurrentPosition() > armScorePos+degreesToTicks(5)){
                servoGripper.setPosition(gripperOpenedPos);
                break;
            }
        }
        drive.followTrajectory(tuckAway);
    }

    public int degreesToTicks(float degrees){
        int ticks = (int) Math.round(degrees*6.2222);
        return ticks * -1;
    }
    public double ticksToDegrees(int ticks){
        double degrees = (double) ticks/6.2222;
        return degrees * -1;
    }

    public class RedTeamPropDetectionPipeline extends OpenCvPipeline {

        @Override
        public Mat processFrame(Mat input) {
            telemetry.addData("Entered function", "process Frame");
            // Preprocess the frame to detect yellow regions
            Mat redMask = preprocessFrame(input);

            // Find contours of the detected yellow regions
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(redMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // Find the largest blue contour (blob)
            MatOfPoint largestContour = findLargestContour(contours);

            if (largestContour != null) {
                // Draw a red outline around the largest detected object
                Imgproc.drawContours(input, contours, contours.indexOf(largestContour), new Scalar(0, 255, 0), 2);
                // Calculate the centroid of the largest contour
                Moments moments = Imgproc.moments(largestContour);
                cX = moments.get_m10() / moments.get_m00();
                cY = moments.get_m01() / moments.get_m00();

                // Draw a dot at the centroid
                String label = "(" + (int) cX + ", " + (int) cY + ")";
                Imgproc.putText(input, label, new Point(cX + 10, cY), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                Imgproc.circle(input, new Point(cX, cY), 5, new Scalar(0, 255, 0), -1);

            }

            return input;
        }
        private Mat preprocessFrame(Mat frame) {
            telemetry.addData("Entered function", "preprocess frame");
            Mat hsvFrame = new Mat();
            Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_RGB2HSV);

            Scalar lowerRed1 = new Scalar(0, 100, 100);
            Scalar upperRed1 = new Scalar(15, 255, 255);

            Scalar lowerRed2 = new Scalar(160, 100, 100);
            Scalar upperRed2 = new Scalar(180, 255, 255);


            Mat redMask1 = new Mat();
            Mat redMask2 = new Mat();
            Mat redMask = new Mat();

            Core.inRange(hsvFrame, lowerRed1, upperRed1, redMask1);
            Core.inRange(hsvFrame, lowerRed2, upperRed2, redMask2);
//
            Core.add(redMask1, redMask2, redMask);


//            telemetry.

            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
//            Tod: check if I can get rid of some of this (or just hide the computation in init loop)
            Imgproc.morphologyEx(redMask, redMask, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(redMask, redMask, Imgproc.MORPH_CLOSE, kernel);

            return redMask;
        }

        private MatOfPoint findLargestContour(List<MatOfPoint> contours) {
            double maxArea = 0;
            telemetry.addData("Entered function", "finding largest contour");
            MatOfPoint largestContour = null;

            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > maxArea) {
                    maxArea = area;
                    largestContour = contour;
                }
            }
            if(largestContour != null) {
                telemetry.addData("Largest Contour: ", Imgproc.contourArea(largestContour));
            }
            return largestContour;
        }

    }

    public void initOpenCVRed() {

        // Create an instance of the camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // Use OpenCvCameraFactory class from FTC SDK to create camera instance
        controlHubCam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        controlHubCam.setPipeline(new A4.RedTeamPropDetectionPipeline());

        controlHubCam.openCameraDevice();
        controlHubCam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
    }

}
