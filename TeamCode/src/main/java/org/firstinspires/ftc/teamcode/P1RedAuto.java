package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Red P.1 Auto")
public class P1RedAuto extends LinearOpMode {
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

        Pose2d RedP1 = new Pose2d(14, -62, Math.toRadians(90));
        drive.setPoseEstimate(RedP1);

        waitForStart();
        webCam.stopStreaming();

        if (GPTCamera.rightSide == true) {
            //Path Red mark 1
            Trajectory RedP1M1T1 = drive.trajectoryBuilder(RedP1)
                    .lineToLinearHeading(new Pose2d(12, -32, Math.toRadians(180)))
                    .build();

            Trajectory RedP1M1T2 = drive.trajectoryBuilder(RedP1M1T1.end())
                    .lineToLinearHeading(new Pose2d(50, -30, Math.toRadians(0)))
                    .build();

            Trajectory RedP1M1T3 = drive.trajectoryBuilder(RedP1M1T2.end())
                    .back(10)
                    .build();

            Trajectory RedP1M1T4 = drive.trajectoryBuilder(RedP1M1T3.end())
                    .strafeTo(new Vector2d(50, -60))
                    .build();

            drive.followTrajectory(RedP1M1T1);
            drive.followTrajectory(RedP1M1T2);

            robot.AMotorUpDown.setPower(0.7);
            robot.AMotorUpDown.setTargetPosition(1000);
            robot.AMotorUpDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (opModeIsActive() && Math.abs(robot.AMotorUpDown.getCurrentPosition() - robot.AMotorUpDown.getTargetPosition()) < 5) {
            }
            robot.AMotorUpDown.setPower(0);

            robot.ClawRotationServo.setPosition(0.45);

            robot.AMotorOutIn.setPower(0.7);
            robot.AMotorOutIn.setTargetPosition(-1500);
            robot.AMotorOutIn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (opModeIsActive() && Math.abs(robot.AMotorOutIn.getCurrentPosition() - robot.AMotorOutIn.getTargetPosition()) < 5) { }
            robot.AMotorOutIn.setPower(0);


            robot.ClawDropServo.setPosition(0.785);
            drive.followTrajectory(RedP1M1T3);
            drive.followTrajectory(RedP1M1T4);
            drive.turn(Math.toRadians(180));

            robot.AMotorOutIn.setPower(0.7);
            robot.AMotorOutIn.setTargetPosition(-1000);
            robot.AMotorOutIn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (opModeIsActive() && Math.abs(robot.AMotorOutIn.getCurrentPosition() - robot.AMotorOutIn.getTargetPosition()) < 5) { }
            robot.AMotorOutIn.setPower(0);

            robot.AMotorUpDown.setPower(0.7);
            robot.AMotorUpDown.setTargetPosition(-300);
            robot.AMotorUpDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (opModeIsActive() && Math.abs(robot.AMotorUpDown.getCurrentPosition() - robot.AMotorUpDown.getTargetPosition()) < 5) {
            }
            robot.AMotorUpDown.setPower(0);

        } else if (GPTCamera.middleSide == true) {

        } else if (GPTCamera.leftSide == true) {

        } else {

        }
    }
}