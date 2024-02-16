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

    public static final double APRILTAG_BACKDROP_X              = 60.25;
    public static final double APRILTAG_AUDIENCE_WALL_X         = -70.25;
    public static final double BACKDROP_APRILTAG_DELTA_Y        = 6.0;
    public static final Pose2d[] APRILTAG_POSES = new Pose2d[] {
            new Pose2d(APRILTAG_BACKDROP_X, 41.41, 90.0),        // TagId 1
            new Pose2d(APRILTAG_BACKDROP_X, 35.41, 90.0),        // TagId 2
            new Pose2d(APRILTAG_BACKDROP_X, 29.41, 90.0),        // TagId 3
            new Pose2d(APRILTAG_BACKDROP_X, -29.41, 90.0),       // TagId 4
            new Pose2d(APRILTAG_BACKDROP_X, -35.41, 90.0),       // TagId 5
            new Pose2d(APRILTAG_BACKDROP_X, -41.41, 90.0),       // TagId 6
            new Pose2d(APRILTAG_AUDIENCE_WALL_X, -40.63, -90.0), // TagId 7
            new Pose2d(APRILTAG_AUDIENCE_WALL_X, -35.13, -90.0), // TagId 8
            new Pose2d(APRILTAG_AUDIENCE_WALL_X, 35.13, -90.0),  // TagId 9
            new Pose2d(APRILTAG_AUDIENCE_WALL_X, 40.63, -90.0)   // TagId 10
    };

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

//        ExposureControl exposure = visionPortal.getCameraControl((ExposureControl.class));
//        exposure.setMode(ExposureControl.Mode.Manual);
//        exposure.setExposure(15, TimeUnit.MILLISECONDS);

//        GainControl gain = visionPortal.getCameraControl(GainControl.class);
//        gain.setGain(255);


//        tagProcessor.setDecimation(3);
        waitForStart();

        double testy = 0;
        double testx = 0;
        double testAngle = 0;

        while (!isStopRequested() && opModeIsActive()) {
            desiredTag = null;
            List<AprilTagDetection> tagDetections = tagProcessor.getDetections();
            telemetry.addData("#",tagDetections.size());
            for (AprilTagDetection desiredTag : tagDetections) {
                if (desiredTag.metadata == null)continue;

                Pose2d aprilTagPose = APRILTAG_POSES[desiredTag.id - 1];
                double angleA = desiredTag.ftcPose.yaw + 90;
                double angleB = 90 - desiredTag.ftcPose.bearing;
                double sideA = desiredTag.ftcPose.range;

                double angleC = 180 - angleA - angleB;

                double sideB = (sideA * Math.sin(Math.toRadians(angleB))) / Math.sin(Math.toRadians(angleA));

                double sideC = (sideA * Math.sin(Math.toRadians(angleC)))/ Math.sin(Math.toRadians(angleA));

                double t2AngleA = angleA - 180;

                double t2AngleB = 90;

                double t2AngleC = 180 - t2AngleA - t2AngleB;

                double t2SideB = (sideC * Math.sin(Math.toRadians(t2AngleB))) / Math.sin(Math.toRadians(t2AngleC));

                double robotYPosition = angleC;
                double robotXPosition = sideB;
                double robotHeadingDegree = desiredTag.ftcPose.yaw;


                double fieldX = aprilTagPose.getX() - robotYPosition;





                telemetry.addData("Robot Y",robotYPosition);
                telemetry.addData("Robot X",robotXPosition);
                telemetry.addData("Robot Heading",robotHeadingDegree);

                double tagPositionX = desiredTag.metadata.fieldPosition.get(0);
                double tagPositionY = desiredTag.metadata.fieldPosition.get(1);

                telemetry.addData("X", desiredTag.ftcPose.x);
                telemetry.addData("Y", desiredTag.ftcPose.y);
                telemetry.addData("Bearing", desiredTag.ftcPose.bearing);
                telemetry.addData("Yaw", desiredTag.ftcPose.yaw);
                telemetry.addData("Range",desiredTag.ftcPose.range);
                testy = desiredTag.ftcPose.x - tagPositionY + 7; //add right number
                testx = 63 - desiredTag.ftcPose.y; //add right number
                testAngle = desiredTag.ftcPose.yaw; //may be a problem
                if(testAngle < 0) {
                    testAngle += 360;
                }


           telemetry.update();
        }

        Pose2d RedP1 = new Pose2d(testx, testy, Math.toRadians(0));
        drive.setPoseEstimate(RedP1);

        Trajectory TestAuto = drive.trajectoryBuilder(RedP1)
        .lineToLinearHeading(new Pose2d(38, 38, Math.toRadians(0)))
                .build();



    }
    }
}
