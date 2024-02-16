package org.firstinspires.ftc.teamcode.OldAutoClasses;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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

@Autonomous(name = "Red P.2 Auto")
public class P2RedAuto extends LinearOpMode {
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
        GPTCamera detector = new GPTCamera(false, telemetry);
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

        Pose2d RedP2 = new Pose2d(-33, -61, Math.toRadians(90));
        drive.setPoseEstimate(RedP2);

        waitForStart();
        webCam.stopStreaming();

        if (GPTCamera.rightSide == true) {
            //Mark right

            Trajectory RedP2MRT1 = drive.trajectoryBuilder(RedP2)
                    .lineToLinearHeading(new Pose2d(-38, -35, Math.toRadians(0)))
                    .build();

            Trajectory RedP2MRT2 = drive.trajectoryBuilder(RedP2MRT1.end())
                    .forward(14)
                    .build();

            Trajectory RedP2MRT3 = drive.trajectoryBuilder(RedP2MRT2.end())
                    .back(14)
                    .build();

            Trajectory RedP2MRT4 = drive.trajectoryBuilder(RedP2MRT3.end())
                    .lineToLinearHeading(new Pose2d(-33, -58, Math.toRadians(0)))
                    .build();

            Trajectory RedP2MRT5 = drive.trajectoryBuilder(RedP2MRT4.end())
                    .lineToLinearHeading(new Pose2d(5, -58, Math.toRadians(0)))
                    .build();

            Trajectory RedP2MRT6 = drive.trajectoryBuilder(RedP2MRT5.end())
                    .splineToConstantHeading(new Vector2d(46, -35), Math.toRadians(0))
                    .build();

            Trajectory RedP2MRT7 = drive.trajectoryBuilder(RedP2MRT6.end())
                    .back(10)
                    .build();

            Trajectory RedP2MRT8 = drive.trajectoryBuilder(RedP2MRT7.end())
                    .strafeTo(new Vector2d(46, -55))
                    .build();

            drive.followTrajectory(RedP2MRT1);
            drive.followTrajectory(RedP2MRT2);
            drive.followTrajectory(RedP2MRT3);
            drive.followTrajectory(RedP2MRT4);
            drive.followTrajectory(RedP2MRT5);
            drive.followTrajectory(RedP2MRT6);

            robot.AMotorUpDown.setPower(0.9);
            robot.AMotorUpDown.setTargetPosition(1050);
            robot.AMotorUpDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (opModeIsActive() && Math.abs(robot.AMotorUpDown.getCurrentPosition() - robot.AMotorUpDown.getTargetPosition()) > 5) { }
            robot.AMotorUpDown.setPower(0);

            robot.ClawRotationServo.setPosition(0.2);

            robot.AMotorOutIn.setPower(0.9);
            robot.AMotorOutIn.setTargetPosition(-1250);
            robot.AMotorOutIn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (opModeIsActive() && Math.abs(robot.AMotorOutIn.getCurrentPosition() - robot.AMotorOutIn.getTargetPosition()) > 5) { }
            robot.AMotorOutIn.setPower(0);

            robot.ClawDropServo.setPosition(0.785);

            drive.followTrajectory(RedP2MRT7);

            robot.AMotorOutIn.setPower(0.9);
            robot.AMotorOutIn.setTargetPosition(-760);
            robot.AMotorOutIn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (opModeIsActive() && Math.abs(robot.AMotorOutIn.getCurrentPosition() - robot.AMotorOutIn.getTargetPosition()) > 5) { }
            robot.AMotorOutIn.setPower(0);

            drive.followTrajectory(RedP2MRT8);
            drive.turn(Math.toRadians(180));

            robot.ClawRotationServo.setPosition(.38);

            robot.AMotorUpDown.setPower(0.9);
            robot.AMotorUpDown.setTargetPosition(-300);
            robot.AMotorUpDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (opModeIsActive() && Math.abs(robot.AMotorUpDown.getCurrentPosition() - robot.AMotorUpDown.getTargetPosition()) > 5) { }
            robot.AMotorUpDown.setPower(0);

        } else if (GPTCamera.middleSide == true) {
            //Mark middle

            Trajectory RedP2MMT1 = drive.trajectoryBuilder(RedP2)
                    .lineToLinearHeading(new Pose2d(-38, -35, Math.toRadians(90)))
                    .build();

            Trajectory RedP2MMT2 = drive.trajectoryBuilder(RedP2MMT1.end())
                    .forward(10)
                    .build();

            Trajectory RedP2MMT3 = drive.trajectoryBuilder(RedP2MMT2.end())
                    .back(17)
                    .build();

            Trajectory RedP2MMT4 = drive.trajectoryBuilder(RedP2MMT3.end())
                    .lineToLinearHeading(new Pose2d(-33, -58, Math.toRadians(0)))
                    .build();

            Trajectory RedP2MMT5 = drive.trajectoryBuilder(RedP2MMT4.end())
                    .lineToLinearHeading(new Pose2d(5, -58, Math.toRadians(0)))
                    .build();

            Trajectory RedP2MMT6 = drive.trajectoryBuilder(RedP2MMT5.end())
                    .splineToConstantHeading(new Vector2d(46, -30), Math.toRadians(0))
                    .build();

            Trajectory RedP2MMT7 = drive.trajectoryBuilder(RedP2MMT6.end())
                    .back(10)
                    .build();

            Trajectory RedP2MMT8 = drive.trajectoryBuilder(RedP2MMT7.end())
                    .strafeTo(new Vector2d(46, -55))
                    .build();

            drive.followTrajectory(RedP2MMT1);
            drive.followTrajectory(RedP2MMT2);
            drive.followTrajectory(RedP2MMT3);
            drive.followTrajectory(RedP2MMT4);
            drive.followTrajectory(RedP2MMT5);
            drive.followTrajectory(RedP2MMT6);

            robot.AMotorUpDown.setPower(0.9);
            robot.AMotorUpDown.setTargetPosition(1050);
            robot.AMotorUpDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (opModeIsActive() && Math.abs(robot.AMotorUpDown.getCurrentPosition() - robot.AMotorUpDown.getTargetPosition()) > 5) { }
            robot.AMotorUpDown.setPower(0);

            robot.ClawRotationServo.setPosition(0.2);

            robot.AMotorOutIn.setPower(0.9);
            robot.AMotorOutIn.setTargetPosition(-1250);
            robot.AMotorOutIn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (opModeIsActive() && Math.abs(robot.AMotorOutIn.getCurrentPosition() - robot.AMotorOutIn.getTargetPosition()) > 5) { }
            robot.AMotorOutIn.setPower(0);

            robot.ClawDropServo.setPosition(0.785);

            drive.followTrajectory(RedP2MMT7);

            robot.AMotorOutIn.setPower(0.9);
            robot.AMotorOutIn.setTargetPosition(-760);
            robot.AMotorOutIn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (opModeIsActive() && Math.abs(robot.AMotorOutIn.getCurrentPosition() - robot.AMotorOutIn.getTargetPosition()) > 5) { }
            robot.AMotorOutIn.setPower(0);

            drive.followTrajectory(RedP2MMT8);
            drive.turn(Math.toRadians(180));

            robot.ClawRotationServo.setPosition(.38);

            robot.AMotorUpDown.setPower(0.9);
            robot.AMotorUpDown.setTargetPosition(-300);
            robot.AMotorUpDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (opModeIsActive() && Math.abs(robot.AMotorUpDown.getCurrentPosition() - robot.AMotorUpDown.getTargetPosition()) > 5) { }
            robot.AMotorUpDown.setPower(0);

        } else if (GPTCamera.leftSide == true) {
            //Mark left

            Trajectory RedP2MLT1 = drive.trajectoryBuilder(RedP2)
                    .lineToLinearHeading(new Pose2d(-37, -35, Math.toRadians(180)))
                    .build();

            Trajectory RedP2MLT2 = drive.trajectoryBuilder(RedP2MLT1.end())
                    .forward(9)
                    .build();

            Trajectory RedP2MLT3 = drive.trajectoryBuilder(RedP2MLT2.end())
                    .back(9)
                    .build();

            Trajectory RedP2MLT4 = drive.trajectoryBuilder(RedP2MLT3.end())
                    .lineToLinearHeading(new Pose2d(-33, -58, Math.toRadians(0)))
                    .build();

            Trajectory RedP2MLT5 = drive.trajectoryBuilder(RedP2MLT4.end())
                    .lineToLinearHeading(new Pose2d(5, -58, Math.toRadians(0)))
                    .build();

            Trajectory RedP2MLT6 = drive.trajectoryBuilder(RedP2MLT5.end())
                    .splineToConstantHeading(new Vector2d(46, -28), Math.toRadians(0))
                    .build();

            Trajectory RedP2MLT7 = drive.trajectoryBuilder(RedP2MLT6.end())
                    .back(10)
                    .build();

            Trajectory RedP2MLT8 = drive.trajectoryBuilder(RedP2MLT7.end())
                    .strafeTo(new Vector2d(46, -55))
                    .build();

            drive.followTrajectory(RedP2MLT1);
            drive.followTrajectory(RedP2MLT2);
            drive.followTrajectory(RedP2MLT3);
            drive.followTrajectory(RedP2MLT4);
            drive.followTrajectory(RedP2MLT5);
            drive.followTrajectory(RedP2MLT6);

            robot.AMotorUpDown.setPower(0.9);
            robot.AMotorUpDown.setTargetPosition(1050);
            robot.AMotorUpDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (opModeIsActive() && Math.abs(robot.AMotorUpDown.getCurrentPosition() - robot.AMotorUpDown.getTargetPosition()) > 5) { }
            robot.AMotorUpDown.setPower(0);

            robot.ClawRotationServo.setPosition(0.2);

            robot.AMotorOutIn.setPower(0.9);
            robot.AMotorOutIn.setTargetPosition(-1250);
            robot.AMotorOutIn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (opModeIsActive() && Math.abs(robot.AMotorOutIn.getCurrentPosition() - robot.AMotorOutIn.getTargetPosition()) > 5) { }
            robot.AMotorOutIn.setPower(0);

            robot.ClawDropServo.setPosition(0.785);

            drive.followTrajectory(RedP2MLT7);

            robot.AMotorOutIn.setPower(0.9);
            robot.AMotorOutIn.setTargetPosition(-760);
            robot.AMotorOutIn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (opModeIsActive() && Math.abs(robot.AMotorOutIn.getCurrentPosition() - robot.AMotorOutIn.getTargetPosition()) > 5) { }
            robot.AMotorOutIn.setPower(0);

            drive.followTrajectory(RedP2MLT8);
            drive.turn(Math.toRadians(180));

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