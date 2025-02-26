package org.firstinspires.ftc.teamcode.Pipelines;

import android.util.Size;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
@Disabled
@Autonomous(name = "Test Auto Camera")
public class TestApril extends LinearOpMode {
    private AprilTagDetection desiredTag = null;
    public static final double APRILTAG_BACKDROP_X = 60.25;
    public static final double APRILTAG_AUDIENCE_WALL_X = -70.25;
    public static final Pose2d[] APRILTAG_POSES = new Pose2d[] {
            new Pose2d(APRILTAG_BACKDROP_X, 41.41, 0.0),        // TagId 1
            new Pose2d(APRILTAG_BACKDROP_X, 35.41, 0.0),        // TagId 2
            new Pose2d(APRILTAG_BACKDROP_X, 29.41, 0.0),        // TagId 3
            new Pose2d(APRILTAG_BACKDROP_X, -29.41, 0.0),       // TagId 4
            new Pose2d(APRILTAG_BACKDROP_X, -35.41, 0.0),       // TagId 5
            new Pose2d(APRILTAG_BACKDROP_X, -41.41, 0.0),       // TagId 6
            new Pose2d(APRILTAG_AUDIENCE_WALL_X, -40.63, 180.0),// TagId 7
            new Pose2d(APRILTAG_AUDIENCE_WALL_X, -35.13, 180.0),// TagId 8
            new Pose2d(APRILTAG_AUDIENCE_WALL_X, 35.13, 180.0), // TagId 9
            new Pose2d(APRILTAG_AUDIENCE_WALL_X, 40.63, 180.0)  // TagId 10
    };

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

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
                .setCameraResolution(new Size(640, 480))
                .build();

        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
        }

        ExposureControl exposure = visionPortal.getCameraControl((ExposureControl.class)); //?
        exposure.setMode(ExposureControl.Mode.Manual);
        exposure.setExposure(15, TimeUnit.MILLISECONDS);

        GainControl gain = visionPortal.getCameraControl(GainControl.class); //?
        gain.setGain(255);

        tagProcessor.setDecimation(3); //?
        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            desiredTag = null;
            List<AprilTagDetection> tagDetections = tagProcessor.getDetections();
            telemetry.addData("#", tagDetections.size());
            for (AprilTagDetection desiredTag : tagDetections) {
                if (desiredTag.metadata == null) continue;
                Pose2d aprilTagPose = APRILTAG_POSES[desiredTag.id - 1];

                double angleA = desiredTag.ftcPose.yaw + 90;
                double angleB = 90 - desiredTag.ftcPose.bearing;
                double sideA = desiredTag.ftcPose.range;

                double angleC = 180 - angleA - angleB;
                double sideB = (sideA * Math.sin(Math.toRadians(angleB))) / Math.sin(Math.toRadians(angleA)); //April tag y RR x
                double sideC = (sideA * Math.sin(Math.toRadians(angleC))) / Math.sin(Math.toRadians(angleA));

                double t2AngleA = 180 - angleA;
                double t2AngleB = 90;
                double t2AngleC = 180 - t2AngleA - t2AngleB;
                double t2SideB = (sideC * Math.sin(Math.toRadians(t2AngleB))) / Math.sin(Math.toRadians(t2AngleC)); //April tag x RR y

                double robotXPosition = sideB;
                double robotYPosition = t2SideB;

                double fieldX = aprilTagPose.getX() - robotXPosition;
                double fieldY = aprilTagPose.getY() - robotYPosition;
                double robotHeadingDegree = aprilTagPose.getHeading() + desiredTag.ftcPose.yaw;

                telemetry.addData("Robot Y", robotYPosition);
                telemetry.addData("Robot X", robotXPosition);
                telemetry.addData("Robot Heading", robotHeadingDegree);

                telemetry.addData("robot x", sideB);
                telemetry.addData("robot y", t2SideB);
                telemetry.addData("robot Heading", robotHeadingDegree);
                telemetry.update();

                Pose2d RedP2 = new Pose2d(fieldX, fieldY, Math.toRadians(robotHeadingDegree));
                drive.setPoseEstimate(RedP2);
            }
        }
    }
}
