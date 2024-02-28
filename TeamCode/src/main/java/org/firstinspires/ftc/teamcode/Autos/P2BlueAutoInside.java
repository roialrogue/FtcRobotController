package org.firstinspires.ftc.teamcode.Autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.Pipelines.GPTCamera;
import org.firstinspires.ftc.teamcode.Pipelines.PoseStorage;
import org.firstinspires.ftc.teamcode.Pipelines.myGamePad;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Position 2 Blue Auto Inside")
public class P2BlueAutoInside extends LinearOpMode {
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
        GPTCamera detector = new GPTCamera(false, telemetry);
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

        robot.BeltMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BeltMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        robot.closeRight();
        robot.closeLeft();
        robot.wristDown();
        robot.rotateDown();
        robot.slidesTo(50,.3);

        Pose2d BlueP2 = new Pose2d(-38, 61, Math.toRadians(270));
        drive.setPoseEstimate(BlueP2);

        waitForStart();
        webCam.stopStreaming();
        sleep(waitTime);

        if (GPTCamera.rightSide) {
            //on the right side

            Trajectory BlueP2MRT1 = drive.trajectoryBuilder(BlueP2)
                    .lineToLinearHeading(new Pose2d(-36, 36, Math.toRadians(180)))
                    .build();

            TrajectorySequence BlueP2MRT2 = drive.trajectorySequenceBuilder(BlueP2MRT1.end())
                    .lineToLinearHeading(new Pose2d(-33, 10, Math.toRadians(0)))
                    .lineToLinearHeading(new Pose2d(5, 10, Math.toRadians(0)))
                    .build();

            Trajectory BlueP2MRT3 = drive.trajectoryBuilder(BlueP2MRT2.end())
                    .splineToConstantHeading(new Vector2d(52, 26), Math.toRadians(0))
                    .build();



            Trajectory BlueP2MRT4 = drive.trajectoryBuilder(BlueP2MRT2.end())
                    .back(10)
                    .build();

            drive.followTrajectory(BlueP2MRT1);
            robot.openLeft();
            robot.wristUp();
            drive.followTrajectorySequence(BlueP2MRT2);
            robot.closeLeft();
            robot.slidesTo(1200);
            drive.followTrajectory(BlueP2MRT3);
            robot.openRight();
            drive.followTrajectory(BlueP2MRT4);
            robot.closeRight();

            if (cycling) {

            }

            if (parkingInside) {
                TrajectorySequence BlueParking = drive.trajectorySequenceBuilder(BlueP2MRT4.end())
                        .lineToLinearHeading(new Pose2d(48, 12, Math.toRadians(180)))
                        .back(10)
                        .build();

                drive.followTrajectorySequence(BlueParking);
            } else {
                TrajectorySequence BlueParking = drive.trajectorySequenceBuilder(BlueP2MRT4.end())
                        .lineToLinearHeading(new Pose2d(48, 60, Math.toRadians(180)))
                        .back(10)
                        .build();
                drive.followTrajectorySequence(BlueParking);
            }

        } else if(GPTCamera.middleSide) {
            //on the middle side

            Trajectory BlueP2MMT1 = drive.trajectoryBuilder(BlueP2)
                    .lineToLinearHeading(new Pose2d(-36, 14, Math.toRadians(90)))
                    .build();

            TrajectorySequence BlueP2MMT2 = drive.trajectorySequenceBuilder(BlueP2MMT1.end())
                    .lineToLinearHeading(new Pose2d(5, 10, Math.toRadians(0)))
                    .build();

            Trajectory BlueP2MMT3 = drive.trajectoryBuilder(BlueP2MMT2.end())
                    .splineToConstantHeading(new Vector2d(52, 33), Math.toRadians(0))
                    .build();

            Trajectory BlueP2MMT4 = drive.trajectoryBuilder(BlueP2MMT3.end())
                    .back(10)
                    .build();


            drive.followTrajectory(BlueP2MMT1);
            robot.openLeft();
            robot.wristUp();
            drive.followTrajectorySequence(BlueP2MMT2);
            robot.closeLeft();
            robot.slidesTo(1200);
            drive.followTrajectory(BlueP2MMT3);
            robot.openRight();
            drive.followTrajectory(BlueP2MMT4);
            robot.closeRight();

            if (cycling) {

            }

            if (parkingInside) {
                TrajectorySequence BlueParking = drive.trajectorySequenceBuilder(BlueP2MMT4.end())
                        .lineToLinearHeading(new Pose2d(48, 12, Math.toRadians(180)))
                        .back(10)
                        .build();

                drive.followTrajectorySequence(BlueParking);
            } else {
                TrajectorySequence BlueParking = drive.trajectorySequenceBuilder(BlueP2MMT4.end())
                        .lineToLinearHeading(new Pose2d(48, 60, Math.toRadians(180)))
                        .back(10)
                        .build();
                drive.followTrajectorySequence(BlueParking);
            }

        } else if(GPTCamera.leftSide) {
            //on the left side

            Trajectory BlueP2MLT1 = drive.trajectoryBuilder(BlueP2)
                    .lineToLinearHeading(new Pose2d(-32, 36, Math.toRadians(0)))
                    .build();

            TrajectorySequence BlueP2MLT2 = drive.trajectorySequenceBuilder(BlueP2MLT1.end())
                    .lineToLinearHeading(new Pose2d(-36, 10, Math.toRadians(0)))
                    .lineToLinearHeading(new Pose2d(5, 10, Math.toRadians(0)))
                    .build();

            Trajectory BlueP2MLT3 = drive.trajectoryBuilder(BlueP2MLT2.end())
                    .splineToConstantHeading(new Vector2d(52, 40), Math.toRadians(0))
                    .build();

            Trajectory BlueP2MLT4 = drive.trajectoryBuilder(BlueP2MLT3.end())
                    .back(10)
                    .build();

            drive.followTrajectory(BlueP2MLT1);
            robot.openLeft();
            robot.wristUp();
            drive.followTrajectorySequence(BlueP2MLT2);
            robot.slidesTo(1200);
            drive.followTrajectory(BlueP2MLT3);
            robot.closeLeft();
            robot.openRight();
            drive.followTrajectory(BlueP2MLT4);
            robot.closeRight();

            if (cycling) {

            }
            if (parkingInside) {
                TrajectorySequence BlueParking = drive.trajectorySequenceBuilder(BlueP2MLT4.end())
                        .lineToLinearHeading(new Pose2d(48, 12, Math.toRadians(180)))
                        .back(10)
                        .build();

                drive.followTrajectorySequence(BlueParking);
            } else {
                TrajectorySequence BlueParking = drive.trajectorySequenceBuilder(BlueP2MLT4.end())
                        .lineToLinearHeading(new Pose2d(48, 60, Math.toRadians(180)))
                        .back(10)
                        .build();

                drive.followTrajectorySequence(BlueParking);
            }
        } else if(GPTCamera.nonSide) {
            telemetry.addData("You need to wait for the Camera to Initialize", "");
        }
        robot.closeRight();
        robot.closeLeft();
        robot.wristDown();
        robot.rotateDown();
        robot.slidesTo(0);
        while(opModeIsActive() && robot.BeltMotor.isBusy()){ }

        PoseStorage.currentPose = drive.getPoseEstimate();
    }
}