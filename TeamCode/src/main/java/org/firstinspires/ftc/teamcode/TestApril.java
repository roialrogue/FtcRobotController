package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class TestApril extends LinearOpMode {

    @Override
    public void runOpMode() {

        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
            .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640,480)) //bigger the resolution the further it can see but impact performance
                //Running a logitech C920 or C310, I can get 30 FPS with 648x480 resolution and a decimation rate of 3. With these settings, the robot can easilly detect the large Apriltag images from all the way across the field. To reliably detect the small images from 6 feet away I need to drop the decimation to 2, and I get about 25 FPS.
                .build();

        waitForStart();



        while (!isStopRequested() && opModeIsActive()) {

            if (tagProcessor.getDetections().size() > 0);
            AprilTagDetection tag = tagProcessor.getDetections().get(-1); //-1 for the first tag it sees or tag idea for specific one

            telemetry.addData("X", tag.ftcPose.x);
            telemetry.addData("Y", tag.ftcPose.y);
            telemetry.addData("Bearing", tag.ftcPose.bearing);
        }
        telemetry.update();
    }
}
