package org.firstinspires.ftc.teamcode.Autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Pipelines.GPTCamera;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.Pipelines.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Pipelines.myGamePad;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Position 2 Red Auto")
public class ZP2RedAuto extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime();
    Hardware robot = Hardware.getInstance();
    OpenCvCamera webCam;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    private GPTCamera detector;
    boolean editingConfig = true;
    boolean parkingInside = true;
    boolean cycling = true;
    int waitTime = 0;

    enum EditingMode { None, WaitTime, Cycling, ParkingInside}

    public void runOpMode() {
        robot.init(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        myGamePad myGamepad = new myGamePad(gamepad1);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        GPTCamera detector = new GPTCamera(true, telemetry);
        webCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webCam.openCameraDevice();
        FtcDashboard.getInstance().startCameraStream(webCam, 0);
        webCam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);

        webCam.setPipeline(detector);

        EditingMode editingMode = EditingMode.None;

        while (editingConfig) {
            if (myGamepad.isXPressed()) {
                switch( editingMode ) {
                    case None: editingMode = EditingMode.WaitTime; break;
                    case WaitTime: editingMode = EditingMode.Cycling; break;
                    case Cycling: editingMode = EditingMode.ParkingInside; break;
                    case ParkingInside: editingMode = EditingMode.None; break;
                }
            }

            switch(editingMode) {
                case None:
                    break;
                case WaitTime:
                    if (myGamepad.isRightBumperPressed()) {
                        waitTime = waitTime + 500;
                    } else if (myGamepad.isLeftBumperPressed() && !(waitTime == 0)) {
                        waitTime = waitTime - 500;
                    }
                    break;

                case Cycling:
                    if (myGamepad.isRightBumperPressed()) {
                        cycling = true;
                    } else if (myGamepad.isLeftBumperPressed()) {
                        cycling = false;
                    }
                    break;

                case ParkingInside:
                    if (myGamepad.isRightBumperPressed()) {
                        parkingInside = true;
                    } else if (myGamepad.isLeftBumperPressed()) {
                        parkingInside = false;
                    }
                    break;
            }
            telemetry.addData("Time the robot is waiting", waitTime);
            telemetry.addData("Cycling", cycling);
            telemetry.addData("Parking on the inside", parkingInside);
            telemetry.update();

            if (myGamepad.isleftstickbuttonPressed()) {
                editingConfig = false;
            }
        }

        telemetry.addData("Ready","Editing auto is done");
        telemetry.update();

        robot.closeRight();
        robot.closeLeft();
        robot.wristDown();
        robot.rotateDown();

        Pose2d RedP2 = new Pose2d(-38, 61, Math.toRadians(270));
        drive.setPoseEstimate(RedP2);

        waitForStart();
        webCam.stopStreaming();
        sleep(waitTime);

        if (GPTCamera.rightSide) {
            //on the right side

            Trajectory RedP2MRT1 = drive.trajectoryBuilder(RedP2)
                    .lineToLinearHeading(new Pose2d(-32, -33, Math.toRadians(0)))
                    .build();

            TrajectorySequence RedP2MRT2 = drive.trajectorySequenceBuilder(RedP2MRT1.end())
                    .lineToLinearHeading(new Pose2d(-36, -58, Math.toRadians(0)))
                    .lineToLinearHeading(new Pose2d(5, -58, Math.toRadians(0)))
                    .splineToConstantHeading(new Vector2d(46, -35), Math.toRadians(0))
                    .build();

            Trajectory RedP2MRT3 = drive.trajectoryBuilder(RedP2MRT2.end())
                    .back(10)
                    .build();

            drive.followTrajectory(RedP2MRT1);
            robot.openLeft();
            robot.closeLeft();
            drive.followTrajectorySequence(RedP2MRT2);
            robot.openRight();
            robot.closeRight();
            drive.followTrajectory(RedP2MRT3);

            if (cycling) {

            }

            if (parkingInside) {
                TrajectorySequence RedParking = drive.trajectorySequenceBuilder(RedP2MRT3.end())
                        .lineToLinearHeading(new Pose2d(48, -12, Math.toRadians(180)))
                        .back(10)
                        .build();

                drive.followTrajectorySequence(RedParking);
            } else {
                TrajectorySequence RedParking = drive.trajectorySequenceBuilder(RedP2MRT3.end())
                        .lineToLinearHeading(new Pose2d(48, -60, Math.toRadians(180)))
                        .back(10)
                        .build();
                drive.followTrajectorySequence(RedParking);
            }

        } else if(GPTCamera.middleSide) {
            //on the middle side

            Trajectory RedP2MMT1 = drive.trajectoryBuilder(RedP2)
                    .lineToLinearHeading(new Pose2d(-40, -33, Math.toRadians(90)))
                    .build();

            Trajectory RedP2MMT2 = drive.trajectoryBuilder(RedP2MMT1.end())
                    .lineToLinearHeading(new Pose2d(-33, -58, Math.toRadians(0)))
                    .lineToLinearHeading(new Pose2d(5, -58, Math.toRadians(0)))
                    .splineToConstantHeading(new Vector2d(46, -35), Math.toRadians(0))
                    .build();

            Trajectory RedP2MMT3 = drive.trajectoryBuilder(RedP2MMT2.end())
                    .back(10)
                    .build();

            drive.followTrajectory(RedP2MMT1);
            robot.openLeft();
            robot.closeLeft();
            drive.followTrajectory(RedP2MMT2);
            robot.openRight();
            robot.closeRight();
            drive.followTrajectory(RedP2MMT3);

            if (cycling) {

            }

            if (parkingInside) {
                TrajectorySequence RedParking = drive.trajectorySequenceBuilder(RedP2MMT3.end())
                        .lineToLinearHeading(new Pose2d(48, -12, Math.toRadians(180)))
                        .back(10)
                        .build();

                drive.followTrajectorySequence(RedParking);
            } else {
                TrajectorySequence RedParking = drive.trajectorySequenceBuilder(RedP2MMT3.end())
                        .lineToLinearHeading(new Pose2d(48, -60, Math.toRadians(180)))
                        .back(10)
                        .build();
                drive.followTrajectorySequence(RedParking);
            }

        } else if(GPTCamera.leftSide) {
            //on the left side

            Trajectory RedP2MLT1 = drive.trajectoryBuilder(RedP2)
                    .lineToLinearHeading(new Pose2d(-38, -33, Math.toRadians(180)))
                    .build();

            Trajectory RedP2MLT2 = drive.trajectoryBuilder(RedP2MLT1.end())
                    .lineToLinearHeading(new Pose2d(-33, -58, Math.toRadians(0)))

                    .lineToLinearHeading(new Pose2d(5, -58, Math.toRadians(0)))

                    .splineToConstantHeading(new Vector2d(46, -30), Math.toRadians(0))
                    .build();

            Trajectory RedP2MLT3 = drive.trajectoryBuilder(RedP2MLT2.end())
                    .back(10)
                    .build();

            drive.followTrajectory(RedP2MLT1);
            robot.openLeft();
            robot.closeLeft();
            drive.followTrajectory(RedP2MLT2);
            robot.openRight();
            robot.closeRight();
            drive.followTrajectory(RedP2MLT3);

            if (cycling) {

            }
            if (parkingInside) {
                TrajectorySequence RedParking = drive.trajectorySequenceBuilder(RedP2MLT3.end())
                        .lineToLinearHeading(new Pose2d(48, -12, Math.toRadians(180)))
                        .back(10)
                        .build();

                drive.followTrajectorySequence(RedParking);
            } else {
                TrajectorySequence RedParking = drive.trajectorySequenceBuilder(RedP2MLT3.end())
                        .lineToLinearHeading(new Pose2d(48, -60, Math.toRadians(180)))
                        .back(10)
                        .build();

                drive.followTrajectorySequence(RedParking);
            }
        } else if(GPTCamera.nonSide) {
            telemetry.addData("You need to wait for the Camera to Initialize", "");
        }


        PoseStorage.currentPose = drive.getPoseEstimate();
    }
}