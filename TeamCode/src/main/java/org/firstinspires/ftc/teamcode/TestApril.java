package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.concurrent.TimeUnit;

public class TestApril extends LinearOpMode {

    @Override
    public void runOpMode() {

        //int[] portalsList = VisionPortal.makeMultiPortalView(2,VisionPortal.MultiPortalLayout.HORIZONTAL);

        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                //.setLensIntrinsics(0,0,0,0)
            .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640,480)) //bigger the resolution the further it can see but impact performance
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
//                .enableCameraMonitoring(Ture)
//                .setCameraMonitorViewId(portalsList[1]) // 2 for second Camera
                .build();

        visionPortal.getCameraState();

        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {}

        ExposureControl exposure = visionPortal.getCameraControl((ExposureControl.class));
        exposure.setMode(ExposureControl.Mode.Manual);
        exposure.setExposure(15, TimeUnit.MILLISECONDS);

        GainControl gain = visionPortal.getCameraControl(GainControl.class);
        //gain.setGain(255);


        tagProcessor.setDecimation(3);
        waitForStart();



        while (!isStopRequested() && opModeIsActive()) {
            if (tagProcessor.getDetections().size() > 0);
            AprilTagDetection tag = tagProcessor.getDetections().get(-1); //-1 for the first tag it sees or tag idea for specific one


            telemetry.addLine(String.format("XYBearing %6.2f %6.2f %6.2f", tag.ftcPose.x, tag.ftcPose.y, tag.ftcPose.bearing));

            tag.metadata.fieldPosition.get(0); // 0 = x, 1 = y, 2 = z
            telemetry.addData("tag position x",tag.metadata.fieldPosition.get(0));
            telemetry.addData("tag position y",tag.metadata.fieldPosition.get(1));

            telemetry.addData("exposure", exposure.isExposureSupported());
            telemetry.addData("gain Max", gain.getMaxGain());
            telemetry.addData("gain Min", gain.getMinGain());

            telemetry.addData("X", tag.ftcPose.x);
            telemetry.addData("Y", tag.ftcPose.y);
            telemetry.addData("Bearing", tag.ftcPose.bearing);
        }
        telemetry.update();
    }
}
