package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "Test Auto Camera")
public class TestApril extends LinearOpMode {

    int DESIRED_TAG_ID = 2;
    private AprilTagDetection desiredTag = null;

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //int[] portalsList = VisionPortal.makeMultiPortalView(2,VisionPortal.MultiPortalLayout.HORIZONTAL);

        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setLensIntrinsics(814.353,814.353,339.689,224.275)
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480)) //bigger the resolution the further it can see but impact performance
//                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
//                .enableCameraMonitoring(Ture)
//                .setCameraMonitorViewId(portalsList[1]) // 2 for second Camera
                .build();

        //visionPortal.getCameraState();

        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
        }

        ExposureControl exposure = visionPortal.getCameraControl((ExposureControl.class));
        exposure.setMode(ExposureControl.Mode.Manual);
        exposure.setExposure(15, TimeUnit.MILLISECONDS);

        GainControl gain = visionPortal.getCameraControl(GainControl.class);
        //gain.setGain(255);


        tagProcessor.setDecimation(3);
        waitForStart();

        double testy = 0;
        double testx = 0;
        double testAngle = 0;

        while (!isStopRequested() && opModeIsActive()) {
            desiredTag = null;
            List<AprilTagDetection> tagDetections = tagProcessor.getDetections();
            telemetry.addData("#",tagDetections.size());
            for (AprilTagDetection detection : tagDetections) {
                if ((detection.metadata != null) &&
                        ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID))  ){
                    desiredTag = detection;
                    break;  // don't look any further.
                } else {
                    telemetry.addData("Unknown Target", "Tag ID %d is not in TagLibrary\n", detection.id);
                }
                telemetry.update();
            }


            if(desiredTag != null) {
                telemetry.addLine(String.format("XYBearing %6.2f %6.2f %6.2f", desiredTag.ftcPose.x, desiredTag.ftcPose.y, desiredTag.ftcPose.bearing));

                double tagPositionX = desiredTag.metadata.fieldPosition.get(0);
                double tagPositionY = desiredTag.metadata.fieldPosition.get(1);

                desiredTag.metadata.fieldPosition.get(0); // 0 = x, 1 = y, 2 = z
                telemetry.addData("tag position x", tagPositionX );
                telemetry.addData("tag position y", tagPositionY );

                telemetry.addData("exposure", exposure.isExposureSupported());
                telemetry.addData("gain Max", gain.getMaxGain());
                telemetry.addData("gain Min", gain.getMinGain());
                telemetry.addData("X", desiredTag.ftcPose.x);
                telemetry.addData("Y", desiredTag.ftcPose.y);
                telemetry.addData("Bearing", desiredTag.ftcPose.bearing);
                telemetry.addData("Yaw", desiredTag.ftcPose.yaw);
                testy = desiredTag.ftcPose.x - tagPositionY + 7; //add right number
                testx = 63 - desiredTag.ftcPose.y; //add right number
                testAngle = desiredTag.ftcPose.yaw; //may be a problem
                if(testAngle < 0) {
                    testAngle += 360;
                }
                telemetry.addData("testyaw", testAngle);
                telemetry.addData("testy", testy);
                telemetry.addData("testx", testx);


                telemetry.update();
            }
        }

        Pose2d RedP1 = new Pose2d(testx, testy, Math.toRadians(0));
        drive.setPoseEstimate(RedP1);

        Trajectory TestAuto = drive.trajectoryBuilder(RedP1)
        .lineToLinearHeading(new Pose2d(38, 38, Math.toRadians(0)))
                .build();



    }
}
