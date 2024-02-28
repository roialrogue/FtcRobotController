package org.firstinspires.ftc.teamcode.Autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
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

@Autonomous(name = "Position 1 Red Auto")
public class P1RedAuto extends LinearOpMode {
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

        robot.BeltMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BeltMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        robot.closeRight();
        robot.closeLeft();
        robot.wristDown();
        robot.rotateDown();
        robot.slidesTo(50,.3);

        Pose2d RedP1 = new Pose2d(15, -61, Math.toRadians(90));
        drive.setPoseEstimate(RedP1);

        waitForStart();
        webCam.stopStreaming();
        sleep(waitTime);

        if (GPTCamera.rightSide) {
            //on the right side

            Trajectory RedP1MRT1 = drive.trajectoryBuilder(RedP1)
                    .lineToLinearHeading(new Pose2d(30, -37, Math.toRadians(140)))
                    .build();

            Trajectory RedP1MRT2 = drive.trajectoryBuilder(RedP1MRT1.end())
                    .lineToLinearHeading(new Pose2d(52, -37, Math.toRadians(0) - 1e-6))
                    .build();

            Trajectory RedP1MRT3 = drive.trajectoryBuilder(RedP1MRT2.end())
                    .back(10)
                    .build();

            drive.followTrajectory(RedP1MRT1);
            robot.openLeft();
            robot.slidesTo(1100);
            robot.wristUp();
            drive.followTrajectory(RedP1MRT2);
            robot.closeLeft();
            robot.openRight();
            drive.followTrajectory(RedP1MRT3);
            robot.closeRight();

            if (cycling) {

            }

            if (parkingInside) {
                TrajectorySequence RedParking = drive.trajectorySequenceBuilder(RedP1MRT3.end())
                        .lineToLinearHeading(new Pose2d(48, -10, Math.toRadians(180)))
                        .back(13)
                        .build();

                drive.followTrajectorySequence(RedParking);
            } else {
                TrajectorySequence RedParking = drive.trajectorySequenceBuilder(RedP1MRT3.end())
                        .lineToLinearHeading(new Pose2d(48, -57, Math.toRadians(180)))
                        .back(15)
                        .build();

                drive.followTrajectorySequence(RedParking);
            }

            robot.closeRight();
            robot.closeLeft();
            robot.wristDown();
            robot.rotateDown();
            robot.slidesTo(0);
            while(opModeIsActive() && robot.BeltMotor.isBusy()){ }
        } else if(GPTCamera.middleSide) {
            //on the middle side

            Trajectory RedP1MMT1 = drive.trajectoryBuilder(RedP1)
                    .lineToLinearHeading(new Pose2d(10, -32, Math.toRadians(90)))
                    .build();

            Trajectory RedP1MMT2 = drive.trajectoryBuilder(RedP1MMT1.end())
                    .lineToLinearHeading(new Pose2d(52, -31, Math.toRadians(0) - 1e-6))
                    .build();

            Trajectory RedP1MMT3 = drive.trajectoryBuilder(RedP1MMT2.end())
                    .back(10)
                    .build();

            drive.followTrajectory(RedP1MMT1);
            robot.openLeft();
            robot.slidesTo(1100);
            robot.wristUp();
            drive.followTrajectory(RedP1MMT2);
            robot.closeLeft();
            robot.openRight();
            drive.followTrajectory(RedP1MMT3);
            robot.closeRight();

            if (cycling) {

            }

            if (parkingInside) {
                TrajectorySequence RedParking = drive.trajectorySequenceBuilder(RedP1MMT3.end())
                        .lineToLinearHeading(new Pose2d(48, -10, Math.toRadians(180)))
                        .back(13)
                        .build();

                drive.followTrajectorySequence(RedParking);
            } else {
                TrajectorySequence RedParking = drive.trajectorySequenceBuilder(RedP1MMT3.end())
                        .lineToLinearHeading(new Pose2d(48, -57, Math.toRadians(180)))
                        .back(15)
                        .build();

                drive.followTrajectorySequence(RedParking);
            }

            robot.closeRight();
            robot.closeLeft();
            robot.wristDown();
            robot.rotateDown();
            robot.slidesTo(0);
            while(opModeIsActive() && robot.BeltMotor.isBusy()){ }
        } else if(GPTCamera.leftSide) {
            //on the left side

            Trajectory RedP1MLT1 = drive.trajectoryBuilder(RedP1)
                    .lineToLinearHeading(new Pose2d(10, -33, Math.toRadians(180)))
                    .build();

            Trajectory RedP1MLT2 = drive.trajectoryBuilder(RedP1MLT1.end())
                    .lineToLinearHeading(new Pose2d(52, -23, Math.toRadians(0) + 1e-6))
                    .build();

            Trajectory RedP1MLT3 = drive.trajectoryBuilder(RedP1MLT2.end())
                    .back(10)
                    .build();

            drive.followTrajectory(RedP1MLT1);
            robot.openLeft();
            robot.slidesTo(1100);
            robot.wristUp();
            drive.followTrajectory(RedP1MLT2);
            robot.closeLeft();
            robot.openRight();
            drive.followTrajectory(RedP1MLT3);
            robot.closeRight();

            if (cycling) {

            }
            if (parkingInside) {
                TrajectorySequence RedParking = drive.trajectorySequenceBuilder(RedP1MLT3.end())
                        .lineToLinearHeading(new Pose2d(48, -10, Math.toRadians(180)))
                        .back(13)
                        .build();

                drive.followTrajectorySequence(RedParking);
            } else {
                TrajectorySequence RedParking = drive.trajectorySequenceBuilder(RedP1MLT3.end())
                        .lineToLinearHeading(new Pose2d(48, -57, Math.toRadians(180)))
                        .back(15)
                        .build();

                drive.followTrajectorySequence(RedParking);
            }

            robot.closeRight();
            robot.closeLeft();
            robot.wristDown();
            robot.rotateDown();
            robot.slidesTo(0);
            while(opModeIsActive() && robot.BeltMotor.isBusy()){ }
        } else if(GPTCamera.nonSide) {
            telemetry.addData("You need to wait for the Camera to Initialize", "");
        }

        PoseStorage.currentPose = drive.getPoseEstimate();
    }
}
