package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
//calibration may be needed for cam: https://www.youtube.com/watch?v=bTcCY3DZM0k download this on windows https://www.3dflow.net/3df-zephyr-free/
@TeleOp
public class aprilTagTest extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException{
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
        waitForStart();
        while(!isStopRequested() && opModeIsActive()){
            if(tagProcessor.getDetections().size()>0){ //make sure it sees at least 1 tag
                AprilTagDetection tag = tagProcessor.getDetections().get(0);
                telemetry.addData("id",tag.id);
                telemetry.addData("x",tag.ftcPose.x);
                telemetry.addData("y",tag.ftcPose.y);
                telemetry.addData("z",tag.ftcPose.z);
                telemetry.addData("roll",tag.ftcPose.roll);
                telemetry.addData("pitch",tag.ftcPose.pitch);
                telemetry.addData("yaw",tag.ftcPose.yaw);
            }
        }
    }
}
