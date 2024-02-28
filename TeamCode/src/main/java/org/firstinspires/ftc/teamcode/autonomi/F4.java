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
@Autonomous(name="F4 (Red Team Close to Board")
public class F4 extends LinearOpMode {

    DcMotor motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight, motorIntake,
            motorArm;

    Servo servoAirplaneTrigger, servoAttackAngle, servoFinger, servoPurpleDepositor;
    int teamPropPlacement = 2;
    double cX = 0;
    double cY = 0;

    private OpenCvCamera controlHubCam;  // Use OpenCvCamera class from FTC SDK
    private static final int CAMERA_WIDTH = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution

    public int TeamPropPositionCalculator(){
        initOpenCVRed();
        telemetry.update();
        if(cX < 100){
            telemetry.addData("Team Prop Location: ", "Left");
            return 0; //Left
        } else if(cX < 400){
            telemetry.addData("Team Prop Location: ", "Center");
            return 1; //center
        } else{
            telemetry.addData("Team Prop Location: ", "Right");
            return 2; //right
        }
    }
    @Override
    public void runOpMode() throws InterruptedException {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);

        motorIntake = hardwareMap.get(DcMotor.class, "IntakeMotor");
        motorArm = hardwareMap.get(DcMotor.class, "ArmMotor");
        motorArm.setDirection(DcMotor.Direction.REVERSE);

        //Servo Hardware Maps
        servoAirplaneTrigger = hardwareMap.get(Servo.class, "AirplaneTriggerServo");
        servoFinger = hardwareMap.get(Servo.class, "GripperServo");
        servoAttackAngle = hardwareMap.get(Servo.class, "WristServo");
        servoPurpleDepositor = hardwareMap.get(Servo.class, "PurpleDepositorServo");
        servoAirplaneTrigger.setDirection(Servo.Direction.REVERSE);
        servoAirplaneTrigger.getController().pwmEnable();
        servoFinger.getController().pwmEnable();
        servoAttackAngle.getController().pwmEnable();
        servoPurpleDepositor.getController().pwmEnable();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d starting_position = new Pose2d(-56.8, -11.6, Math.PI);
        Trajectory rightProp = drive.trajectoryBuilder(starting_position)
                .splineTo(new Vector2d(-51, -11.64), 0)
                .splineTo(new Vector2d(-46, -21.4), 3.0/2*Math.PI)
                .build();
        Trajectory rightBackboard = drive.trajectoryBuilder(rightProp.end())
                .splineTo(new Vector2d(-50, -21.4), 0)
                .splineTo(new Vector2d(-36.2, -46), Math.PI/2)
                .build();
        Trajectory centerProp = drive.trajectoryBuilder(starting_position)
                .splineTo(new Vector2d(-26.3, -24.1), 0)
                .build();
        Trajectory leftProp = drive.trajectoryBuilder(starting_position)
                .splineTo(new Vector2d(-36.7, 0), 0)
                .splineTo(new Vector2d(-36.7, -11.6), 0)
                .build();


        while(opModeInInit()){
//            teamPropPlacement = 2;
        }
        waitForStart();
        drive.setPoseEstimate(starting_position);
        if(teamPropPlacement == 0){
            drive.followTrajectory(leftProp);
        } else if(teamPropPlacement == 1){
            drive.followTrajectory(centerProp);
        } else{
            drive.followTrajectory(rightProp);
            servoPurpleDepositor.setPosition(1);
            sleep(3000);
            drive.followTrajectory(rightBackboard);
        }
    }

    public class RedTeamPropDetectionPipeline extends OpenCvPipeline {

        @Override
        public Mat processFrame(Mat input) {
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
            Mat hsvFrame = new Mat();
            Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_RGB2HSV);

            Scalar lowerRed1 = new Scalar(0, 100, 100);
            Scalar upperRed1 = new Scalar(20, 255, 255);

            Scalar lowerRed2 = new Scalar(235, 100, 100);
            Scalar upperRed2 = new Scalar(255, 255, 255);


            Mat redMask1 = new Mat();
            Mat redMask2 = new Mat();

            Core.inRange(hsvFrame, lowerRed1, upperRed1, redMask1);
            Core.inRange(hsvFrame, lowerRed2, upperRed2, redMask2);

            Mat redMask = new Mat();

            Core.add(redMask1, redMask2, redMask);


            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
//            Tod: check if I can get rid of some of this (or just hide the computation in init loop)
            Imgproc.morphologyEx(redMask, redMask, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(redMask, redMask, Imgproc.MORPH_CLOSE, kernel);

            return redMask;
        }

        private MatOfPoint findLargestContour(List<MatOfPoint> contours) {
            double maxArea = 0;
            MatOfPoint largestContour = null;

            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > maxArea) {
                    maxArea = area;
                    largestContour = contour;
                }
            }
            if(largestContour != null) {
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

        controlHubCam.setPipeline(new F4.RedTeamPropDetectionPipeline());

        controlHubCam.openCameraDevice();
        controlHubCam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
    }

}
