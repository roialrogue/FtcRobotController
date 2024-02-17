package org.firstinspires.ftc.teamcode.Pipelines;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

//import org.ejml.dense.row.CommonOps_DDRM;

//import org.firstinspires.ftc.teamcode.AveragingArray;



/*
 * This OpMode illustrates the basics of AprilTag recognition and pose estimation, using
 * the easy way.
 *
 * For an introduction to AprilTags, see the FTC-DOCS link below:
 * https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/apriltag_intro/apriltag-intro.html
 *
 * In this sample, any visible tag ID will be detected and displayed, but only tags that are included in the default
 * "TagLibrary" will have their position and orientation information displayed.  This default TagLibrary contains
 * the current Season's AprilTags and a small set of "test Tags" in the high number range.
 *
 * When an AprilTag in the TagLibrary is detected, the SDK provides location and orientation of the tag, relative to the camera.
 * This information is provided in the "ftcPose" member of the returned "detection", and is explained in the ftc-docs page linked below.
 * https://ftc-docs.firstinspires.org/apriltag-detection-values
 *
 * To experiment with using AprilTags to navigate, try out these two driving samples:
 * RobotAutoDriveToAprilTagOmni and RobotAutoDriveToAprilTagTank
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */


@TeleOp(name = "ApriltagAbsolutePOS", group = "Concept")
//@Disabled
public class ApriltagAbsolutePOS extends LinearOpMode {


    //Arrays not used
//    AveragingArray xAverageArray = new AveragingArray(10);
//    AveragingArray yAverageArray = new AveragingArray(10);
//    AveragingArray headingAverageArray = new AveragingArray(10);


    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {

        initAprilTag();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                telemetryAprilTag();

                // Push telemetry to the Driver Station.
                telemetry.update();


                // Share the CPU.
                sleep(20);
            }
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

    }   // end method runOpMode()

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {

        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        //aprilTag.setDecimation(); //larger=higher FPS, less range
        //aprilTag.setPoseSolver(); //experiment with

        // Create the vision portal the easy way.
        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, aprilTag);
        }


    }   // end method initAprilTag()

    /**
     * Add telemetry about AprilTag detections.
     */
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {


                // telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", absoluteX, absoluteY));
                //telemetry.addLine(String.valueOf(absoluteX));
                //telemetry.addLine(String.valueOf(absoluteY));
                //telemetry.addLine(String.valueOf(heading));


                //telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                //telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                //telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                //telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));

                //CameraPosition cameraPositionInfo = calculateCameraPosition(detection.rawPose., detection.ftcPose.bearing, detection.ftcPose.yaw);

                //CameraPosition cameraPositionInfo = calculateCameraPosition(detection.rawPose.x, detection.rawPose.y, detection.rawPose.z);


                Orientation rot = Orientation.getOrientation(detection.rawPose.R, AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);


                //telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (x, y, deg)", detection.rawPose.x, detection.rawPose.y, detection.rawPose.z));
                //telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (x, y, deg)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                // telemetry.addLine(String.valueOf(detection.ftcPose.bearing));
                //telemetry.addLine(String.valueOf(detection.ftcPose.y));


                //telemetry.addLine(String.valueOf(sideAC)); //X offset

                //telemetry.addLine(String.valueOf(detection.ftcPose.yaw)); //heading
                //telemetry.addLine(String.valueOf(detection.ftcPose.x)); //heading

                //WORKS
                // Hardcoded values for angle A, angle B,of side a
//                double angleA = detection.ftcPose.yaw + 90; // Angle A in degrees
//                double angleB = 90 - detection.ftcPose.bearing; // Angle B in degrees
//                double sideA = detection.ftcPose.range;   // Length of side a
//
//                // Calculating the measure of angle C
//                double angleC = 180 - angleA - angleB;
//
//                // Calculate the length of side b
//                double sideB = (sideA * Math.sin(Math.toRadians(angleB))) / Math.sin(Math.toRadians(angleA));
//
//                // Calculate the length of side c
//                double sideC = (sideA * Math.sin(Math.toRadians(angleC))) / Math.sin(Math.toRadians(angleA));
                //WORKS

                double angleA = detection.ftcPose.yaw + 90;
                double angleB = 90 - detection.ftcPose.bearing;
                double sideA = detection.ftcPose.range;

                double angleC = 180 - angleA - angleB;

                double sideB = (sideA * Math.sin(Math.toRadians(angleB))) / Math.sin(Math.toRadians(angleA));

                double sideC = (sideA * Math.sin(Math.toRadians(angleC)))/ Math.sin(Math.toRadians(angleA));

                double t2AngleA = 180 - angleA;

                double t2AngleB = 90;

                double t2AngleC = 180 - t2AngleA - t2AngleB;

                double t2SideB = (sideC * Math.sin(Math.toRadians(t2AngleB))) / Math.sin(Math.toRadians(t2AngleC));


                telemetry.addLine(String.valueOf(sideC)); //X in relation to apriltag
                telemetry.addLine(String.valueOf(sideB)); //Y in relation to apriltag
                telemetry.addLine(String.valueOf(detection.ftcPose.yaw));


                // Output


                //telemetry.addLine(xAverageArray.toString());
                //telemetry.addLine(yAverageArray.toString());
                //telemetry.addLine(headingAverageArray.toString());


                //telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (x, y, deg)", camX, camY,camZ));
                //telemetry.addLine(String.valueOf(Math.toDegrees(heading)));
                //telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (x, y, deg)",rot.firstAngle,rot.secondAngle,rot.thirdAngle)); //first is x, second is Z third is y(y is roll, x is pitch, z is yaw)((First= Pitch, Second = Yaw, third is roll)


                //telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        //telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        //telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        // telemetry.addLine("RBE = Range, Bearing & Elevation");

    }   // end method telemetryAprilTag()
}