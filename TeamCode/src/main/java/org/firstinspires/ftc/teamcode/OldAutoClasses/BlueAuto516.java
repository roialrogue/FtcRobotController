package org.firstinspires.ftc.teamcode.OldAutoClasses;

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
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "516 Blue Auto")
public class BlueAuto516 extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime();
    Hardware robot = Hardware.getInstance();
    OpenCvCamera webCam;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    boolean isBlue;
    private GPTCamera detector;

    public void runOpMode() {
        robot.init(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        GPTCamera detector = new GPTCamera(true, telemetry);
        webCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webCam.openCameraDevice();
        FtcDashboard.getInstance().startCameraStream(webCam, 0);
        webCam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
        webCam.setPipeline(detector);

        robot.AMotorOutIn.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.AMotorUpDown.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.AMotorOutIn.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.AMotorUpDown.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.ClawRotationServo.setPosition(.38);
        robot.ClawDropServo.setPosition(.5);

        Pose2d BlueP1 = new Pose2d(15, 61, Math.toRadians(270));
        drive.setPoseEstimate(BlueP1);

        waitForStart();
        webCam.stopStreaming();

        if (GPTCamera.rightSide == true) {
            //Mark right

            Trajectory BlueP1MRT1 = drive.trajectoryBuilder(BlueP1)
                    .lineToLinearHeading(new Pose2d(11, 32, Math.toRadians(180)))
                    .build();

            Trajectory BlueP1MRT2 = drive.trajectoryBuilder(BlueP1MRT1.end())
                    .forward(3)
                    .build();

            Trajectory BlueP1MRT3 = drive.trajectoryBuilder(BlueP1MRT2.end())
                    .back(3)
                    .build();

            Trajectory BlueP1MRT4 = drive.trajectoryBuilder(BlueP1MRT3.end())
                    .lineToLinearHeading(new Pose2d(50, 26, Math.toRadians(0)))
                    .build();

            Trajectory BlueP1MRT5 = drive.trajectoryBuilder(BlueP1MRT4.end())
                    .back(10)
                    .build();

            Trajectory BlueP1MRT6 = drive.trajectoryBuilder(BlueP1MRT5.end())
                    .lineToLinearHeading(new Pose2d(48, 9, Math.toRadians(180)))
                    .build();

            Trajectory BlueP1MRT7 = drive.trajectoryBuilder(BlueP1MRT6.end())
                    .back(5)
                    .build();

            drive.followTrajectory(BlueP1MRT1);
            drive.followTrajectory(BlueP1MRT2);
            drive.followTrajectory(BlueP1MRT3);
            drive.followTrajectory(BlueP1MRT4);

            robot.AMotorUpDown.setPower(0.9);
            robot.AMotorUpDown.setTargetPosition(1050);
            robot.AMotorUpDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (opModeIsActive() && Math.abs(robot.AMotorUpDown.getCurrentPosition() - robot.AMotorUpDown.getTargetPosition()) > 5) { }
            robot.AMotorUpDown.setPower(0);

            robot.ClawRotationServo.setPosition(0.2);

            robot.AMotorOutIn.setPower(0.9);
            robot.AMotorOutIn.setTargetPosition(-1270);
            robot.AMotorOutIn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (opModeIsActive() && Math.abs(robot.AMotorOutIn.getCurrentPosition() - robot.AMotorOutIn.getTargetPosition()) > 5) { }
            robot.AMotorOutIn.setPower(0);

            robot.ClawDropServo.setPosition(0.785);
            sleep(200);

            drive.followTrajectory(BlueP1MRT5);

            robot.AMotorOutIn.setPower(0.9);
            robot.AMotorOutIn.setTargetPosition(-760);
            robot.AMotorOutIn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (opModeIsActive() && Math.abs(robot.AMotorOutIn.getCurrentPosition() - robot.AMotorOutIn.getTargetPosition()) > 5) { }
            robot.AMotorOutIn.setPower(0);

            drive.followTrajectory(BlueP1MRT6);
            drive.followTrajectory(BlueP1MRT7);

            robot.ClawRotationServo.setPosition(.38);

            robot.AMotorUpDown.setPower(0.9);
            robot.AMotorUpDown.setTargetPosition(-300);
            robot.AMotorUpDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (opModeIsActive() && Math.abs(robot.AMotorUpDown.getCurrentPosition() - robot.AMotorUpDown.getTargetPosition()) > 5) { }
            robot.AMotorUpDown.setPower(0);

        } else if (GPTCamera.middleSide == true) {
            //Mark middle

            Trajectory BlueP1MMT1 = drive.trajectoryBuilder(BlueP1)
                    .lineToLinearHeading(new Pose2d(16, 34, Math.toRadians(270)))
                    .build();

            Trajectory BlueP1MMT2 = drive.trajectoryBuilder(BlueP1MMT1.end())
                    .forward(5)
                    .build();

            Trajectory BlueP1MMT3 = drive.trajectoryBuilder(BlueP1MMT2.end())
                    .back(8)
                    .build();

            Trajectory BlueP1MMT4 = drive.trajectoryBuilder(BlueP1MMT3.end())
                    .lineToLinearHeading(new Pose2d(50, 33, Math.toRadians(0)))
                    .build();

            Trajectory BlueP1MMT5 = drive.trajectoryBuilder(BlueP1MMT4.end())
                    .back(10)
                    .build();

            Trajectory BlueP1MMT6 = drive.trajectoryBuilder(BlueP1MMT5.end())
                    .lineToLinearHeading(new Pose2d(46, 6, Math.toRadians(180)))
                    .build();

            Trajectory BlueP1MMT7 = drive.trajectoryBuilder(BlueP1MMT6.end())
                    .back(10)
                    .build();

            drive.followTrajectory(BlueP1MMT1);
            drive.followTrajectory(BlueP1MMT2);
            drive.followTrajectory(BlueP1MMT3);
            drive.followTrajectory(BlueP1MMT4);

            robot.AMotorUpDown.setPower(0.9);
            robot.AMotorUpDown.setTargetPosition(1100);
            robot.AMotorUpDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (opModeIsActive() && Math.abs(robot.AMotorUpDown.getCurrentPosition() - robot.AMotorUpDown.getTargetPosition()) > 5) { }
            robot.AMotorUpDown.setPower(0);

            robot.ClawRotationServo.setPosition(0.2);

            robot.AMotorOutIn.setPower(0.9);
            robot.AMotorOutIn.setTargetPosition(-1420);
            robot.AMotorOutIn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (opModeIsActive() && Math.abs(robot.AMotorOutIn.getCurrentPosition() - robot.AMotorOutIn.getTargetPosition()) > 5) { }
            robot.AMotorOutIn.setPower(0);

            robot.ClawDropServo.setPosition(0.785);
            sleep(200);

            drive.followTrajectory(BlueP1MMT5);

            robot.AMotorOutIn.setPower(0.9);
            robot.AMotorOutIn.setTargetPosition(-760);
            robot.AMotorOutIn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (opModeIsActive() && Math.abs(robot.AMotorOutIn.getCurrentPosition() - robot.AMotorOutIn.getTargetPosition()) > 5) { }
            robot.AMotorOutIn.setPower(0);

            drive.followTrajectory(BlueP1MMT6);
            drive.followTrajectory(BlueP1MMT7);

            robot.ClawRotationServo.setPosition(.38);

            robot.AMotorUpDown.setPower(0.9);
            robot.AMotorUpDown.setTargetPosition(-300);
            robot.AMotorUpDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (opModeIsActive() && Math.abs(robot.AMotorUpDown.getCurrentPosition() - robot.AMotorUpDown.getTargetPosition()) > 5) { }
            robot.AMotorUpDown.setPower(0);

        } else if (GPTCamera.leftSide == true) {
            //Mark left
            Trajectory BlueP1MLT1 = drive.trajectoryBuilder(BlueP1)
                    .lineToLinearHeading(new Pose2d(30, 42, Math.toRadians(225)))
                    .build();

            Trajectory BlueP1MLT2 = drive.trajectoryBuilder(BlueP1MLT1.end())
                    .forward(5)
                    .build();

            Trajectory BlueP1MLT3 = drive.trajectoryBuilder(BlueP1MLT2.end())
                    .back(5)
                    .build();

            Trajectory BlueP1MLT4 = drive.trajectoryBuilder(BlueP1MLT3.end())
                    .lineToLinearHeading(new Pose2d(50, 40, Math.toRadians(0)))
                    .build();

            Trajectory BlueP1MLT5 = drive.trajectoryBuilder(BlueP1MLT4.end())
                    .back(10)
                    .build();

            Trajectory BlueP1MLT6 = drive.trajectoryBuilder(BlueP1MLT5.end())
                    .lineToLinearHeading(new Pose2d(48, 10, Math.toRadians(180)))
                    .build();

            Trajectory BlueP1MLT7 = drive.trajectoryBuilder(BlueP1MLT6.end())
                    .back(10)
                    .build();

            drive.followTrajectory(BlueP1MLT1);
            drive.followTrajectory(BlueP1MLT2);
            drive.followTrajectory(BlueP1MLT3);
            drive.followTrajectory(BlueP1MLT4);

            robot.AMotorUpDown.setPower(0.9);
            robot.AMotorUpDown.setTargetPosition(1050);
            robot.AMotorUpDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (opModeIsActive() && Math.abs(robot.AMotorUpDown.getCurrentPosition() - robot.AMotorUpDown.getTargetPosition()) > 5) { }
            robot.AMotorUpDown.setPower(0);

            robot.ClawRotationServo.setPosition(0.2);

            robot.AMotorOutIn.setPower(0.9);
            robot.AMotorOutIn.setTargetPosition(-1270);
            robot.AMotorOutIn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (opModeIsActive() && Math.abs(robot.AMotorOutIn.getCurrentPosition() - robot.AMotorOutIn.getTargetPosition()) > 5) { }
            robot.AMotorOutIn.setPower(0);

            robot.ClawDropServo.setPosition(0.785);
            sleep(200);

            drive.followTrajectory(BlueP1MLT5);

            robot.AMotorOutIn.setPower(0.9);
            robot.AMotorOutIn.setTargetPosition(-760);
            robot.AMotorOutIn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (opModeIsActive() && Math.abs(robot.AMotorOutIn.getCurrentPosition() - robot.AMotorOutIn.getTargetPosition()) > 5) { }
            robot.AMotorOutIn.setPower(0);

            drive.followTrajectory(BlueP1MLT6);
            drive.followTrajectory(BlueP1MLT7);

            robot.ClawRotationServo.setPosition(.38);

            robot.AMotorUpDown.setPower(0.9);
            robot.AMotorUpDown.setTargetPosition(-300);
            robot.AMotorUpDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (opModeIsActive() && Math.abs(robot.AMotorUpDown.getCurrentPosition() - robot.AMotorUpDown.getTargetPosition()) > 5) { }
            robot.AMotorUpDown.setPower(0);

        } else {
            telemetry.addData("Wait","Wait");
            telemetry.update();
        }
    }
}