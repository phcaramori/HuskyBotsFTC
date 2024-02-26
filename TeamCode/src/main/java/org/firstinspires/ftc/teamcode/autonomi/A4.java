package org.firstinspires.ftc.teamcode.autonomi;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous(name="A4 (Red Team Close to Board")
public class A4 extends LinearOpMode {

    DcMotor motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight, motorIntake,
            motorArm;

    OpenCvCamera webcam = null;

    int teamPropLocation;

    double frontLeftPower = 0, backLeftPower = 0, frontRightPower = 0, backRightPower = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        motorIntake = hardwareMap.get(DcMotor.class, "IntakeMotor");
        motorArm = hardwareMap.get(DcMotor.class, "ArmMotor");
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorArm.setDirection(DcMotor.Direction.REVERSE);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.setPipeline(new TeamPropDetectionPipeline());

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                Log.d("Webcam Error", String.valueOf(errorCode));
            }
        });

        while (opModeInInit()) {

        }

        waitForStart(); /* Tells robot to do nothing until start is hit */
        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive()) {

        }
    }

    class TeamPropDetectionPipeline extends OpenCvPipeline{
        Mat output = new Mat();
        @Override
        public Mat processFrame(Mat input) {
            output = input;
            return output;
        }
    }
}
