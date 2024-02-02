package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Blue P.1 Auto")
public class P1BlueAuto extends LinearOpMode {
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

        Pose2d BlueP1 = new Pose2d(10, 62, Math.toRadians(270));
        drive.setPoseEstimate(BlueP1);

        waitForStart();
        webCam.stopStreaming();

        if (GPTCamera.rightSide == true) {

        } else if (GPTCamera.middleSide == true) {

        } else if (GPTCamera.leftSide == true) {
            Trajectory BlueP1M3T1 = drive.trajectoryBuilder(BlueP1)
                    .lineToLinearHeading(new Pose2d(30, 42, Math.toRadians(225)))
                    .build();

            Trajectory BlueP1M3T2 = drive.trajectoryBuilder(BlueP1M3T1.end())
                    .forward(12)
                    .build();

            Trajectory BlueP1M3T3 = drive.trajectoryBuilder(BlueP1M3T2.end())
                    .back(12)
                    .build();

            Trajectory BlueP1M3T4 = drive.trajectoryBuilder(BlueP1M3T3.end())
                    .lineToLinearHeading(new Pose2d(44, 42, Math.toRadians(0)))
                    .build();

            Trajectory BlueP1M3T5 = drive.trajectoryBuilder(BlueP1M3T4.end())
                    .back(10)
                    .build();

            Trajectory BlueP1M3T6 = drive.trajectoryBuilder(BlueP1M3T5.end())
                    .strafeTo(new Vector2d(50, 62))
                    .build();

            drive.followTrajectory(BlueP1M3T1);
            drive.followTrajectory(BlueP1M3T2);
            drive.followTrajectory(BlueP1M3T3);

            robot.AMotorUpDown.setPower(0.7);
            robot.AMotorUpDown.setTargetPosition(1000);
            robot.AMotorUpDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (opModeIsActive() && Math.abs(robot.AMotorUpDown.getCurrentPosition() - robot.AMotorUpDown.getTargetPosition()) > 5) {
                telemetry.addData("UpDown", robot.AMotorUpDown.getCurrentPosition());
                telemetry.addData("OutIn", robot.AMotorOutIn.getCurrentPosition());
                telemetry.update();
            }
            robot.AMotorUpDown.setPower(0);

            sleep(2000);

            robot.ClawRotationServo.setPosition(0.45);

            robot.AMotorOutIn.setPower(0.7);
            robot.AMotorOutIn.setTargetPosition(-1500);
            robot.AMotorOutIn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (opModeIsActive() && Math.abs(robot.AMotorOutIn.getCurrentPosition() - robot.AMotorOutIn.getTargetPosition()) > 5) {
                telemetry.addData("UpDown", robot.AMotorUpDown.getCurrentPosition());
                telemetry.addData("OutIn", robot.AMotorOutIn.getCurrentPosition());
                telemetry.update();
            }
            robot.AMotorOutIn.setPower(0);

            sleep(2000);

            robot.ClawDropServo.setPosition(0.785);

            drive.followTrajectory(BlueP1M3T4);
            drive.followTrajectory(BlueP1M3T5);
            drive.followTrajectory(BlueP1M3T6);
            drive.turn(Math.toRadians(180));

            robot.AMotorOutIn.setPower(0.7);
            robot.AMotorOutIn.setTargetPosition(-1000);
            robot.AMotorOutIn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (opModeIsActive() && Math.abs(robot.AMotorOutIn.getCurrentPosition() - robot.AMotorOutIn.getTargetPosition()) > 5) {
                telemetry.addData("UpDown", robot.AMotorUpDown.getCurrentPosition());
                telemetry.addData("OutIn", robot.AMotorOutIn.getCurrentPosition());
                telemetry.update();
            }
            robot.AMotorOutIn.setPower(0);


            sleep(2000);

            robot.AMotorUpDown.setPower(0.7);
            robot.AMotorUpDown.setTargetPosition(-300);
            robot.AMotorUpDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (opModeIsActive() && Math.abs(robot.AMotorUpDown.getCurrentPosition() - robot.AMotorUpDown.getTargetPosition()) > 5) {
                telemetry.addData("UpDown", robot.AMotorUpDown.getCurrentPosition());
                telemetry.addData("OutIn", robot.AMotorOutIn.getCurrentPosition());
                telemetry.update();
            }
            robot.AMotorUpDown.setPower(0);

            sleep(2000);


        } else {

        }
    }
}