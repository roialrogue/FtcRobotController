//package org.firstinspires.ftc.teamcode.OldAutoClasses;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.trajectory.Trajectory;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.Pipelines.GPTCamera;
//import org.firstinspires.ftc.teamcode.Hardware;
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//
//@Autonomous(name = "Red Auto 516")
//public class RedAuto516 extends LinearOpMode {
//    ElapsedTime runtime = new ElapsedTime();
//    Hardware robot = Hardware.getInstance();
//    OpenCvCamera webCam;
//    FtcDashboard dashboard = FtcDashboard.getInstance();
//    boolean isBlue;
//    private GPTCamera detector;
//
//    public void runOpMode() {
//        robot.init(hardwareMap);
//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        GPTCamera detector = new GPTCamera(false, telemetry);
//        webCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//        webCam.openCameraDevice();
//        FtcDashboard.getInstance().startCameraStream(webCam, 0);
//        webCam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
//        webCam.setPipeline(detector);
//
//        robot.AMotorOutIn.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.AMotorUpDown.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.AMotorOutIn.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.AMotorUpDown.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        robot.ClawRotationServo.setPosition(.38);
//        robot.ClawDropServo.setPosition(.5);
//
//        Pose2d RedP1 = new Pose2d(10, -61, Math.toRadians(90));
//        drive.setPoseEstimate(RedP1);
//
//        waitForStart();
//        webCam.stopStreaming();
//
//        if (GPTCamera.rightSide == true) {
//            //mark right
//
//            Trajectory RedP1MLT1 = drive.trajectoryBuilder(RedP1)
//                    .lineToLinearHeading(new Pose2d(30, -44, Math.toRadians(140)))
//                    .build();
//
//            Trajectory RedP1MLT2 = drive.trajectoryBuilder(RedP1MLT1.end())
//                    .forward(12)
//                    .build();
//
//            Trajectory RedP1MLT3 = drive.trajectoryBuilder(RedP1MLT2.end())
//                    .back(12)
//                    .build();
//
//            Trajectory RedP1MLT4 = drive.trajectoryBuilder(RedP1MLT3.end())
//                    .lineToLinearHeading(new Pose2d(46, -35, Math.toRadians(0)))
//                    .build();
//
//            Trajectory RedP1MLT5 = drive.trajectoryBuilder(RedP1MLT4.end())
//                    .back(10)
//                    .build();
//
//            Trajectory RedP1MLT6 = drive.trajectoryBuilder(RedP1MLT5.end())
//                    .lineToLinearHeading(new Pose2d(44, -10, Math.toRadians(180)))
//                    .build();
//
//            Trajectory RedP1MLT7 = drive.trajectoryBuilder(RedP1MLT6.end())
//                    .back(10)
//                    .build();
//
//            drive.followTrajectory(RedP1MLT1);
//            drive.followTrajectory(RedP1MLT2);
//            drive.followTrajectory(RedP1MLT3);
//            drive.followTrajectory(RedP1MLT4);
//
//            robot.AMotorUpDown.setPower(0.9);
//            robot.AMotorUpDown.setTargetPosition(1100);
//            robot.AMotorUpDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            while (opModeIsActive() && Math.abs(robot.AMotorUpDown.getCurrentPosition() - robot.AMotorUpDown.getTargetPosition()) > 5) { }
//            robot.AMotorUpDown.setPower(0);
//
//            robot.ClawRotationServo.setPosition(0.2);
//
//            robot.AMotorOutIn.setPower(0.9);
//            robot.AMotorOutIn.setTargetPosition(-1270);
//            robot.AMotorOutIn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            while (opModeIsActive() && Math.abs(robot.AMotorOutIn.getCurrentPosition() - robot.AMotorOutIn.getTargetPosition()) > 5) { }
//            robot.AMotorOutIn.setPower(0);
//
//            robot.ClawDropServo.setPosition(0.785);
//            sleep(200);
//
//            drive.followTrajectory(RedP1MLT5);
//
//            robot.AMotorOutIn.setPower(0.9);
//            robot.AMotorOutIn.setTargetPosition(-760);
//            robot.AMotorOutIn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            while (opModeIsActive() && Math.abs(robot.AMotorOutIn.getCurrentPosition() - robot.AMotorOutIn.getTargetPosition()) > 5) { }
//            robot.AMotorOutIn.setPower(0);
//
//            drive.followTrajectory(RedP1MLT6);
//            drive.followTrajectory(RedP1MLT7);
//
//            robot.ClawRotationServo.setPosition(.38);
//
//            robot.AMotorUpDown.setPower(0.9);
//            robot.AMotorUpDown.setTargetPosition(-300);
//            robot.AMotorUpDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            while (opModeIsActive() && Math.abs(robot.AMotorUpDown.getCurrentPosition() - robot.AMotorUpDown.getTargetPosition()) > 5) { }
//            robot.AMotorUpDown.setPower(0);
//
//        } else if (GPTCamera.middleSide == true) {
//            //Mark middle
//
//            Trajectory RedP1MMT1 = drive.trajectoryBuilder(RedP1)
//                    .lineToLinearHeading(new Pose2d(10, -34, Math.toRadians(90)))
//                    .build();
//
//            Trajectory RedP1MMT2 = drive.trajectoryBuilder(RedP1MMT1.end())
//                    .forward(5)
//                    .build();
//
//            Trajectory RedP1MMT3 = drive.trajectoryBuilder(RedP1MMT2.end())
//                    .back(9)
//                    .build();
//
//            Trajectory RedP1MMT4 = drive.trajectoryBuilder(RedP1MMT3.end())
//                    .lineToLinearHeading(new Pose2d(46, -27.5, Math.toRadians(0)))
//                    .build();
//
//            Trajectory RedP1MMT5 = drive.trajectoryBuilder(RedP1MMT4.end())
//                    .back(10)
//                    .build();
//
//            Trajectory RedP1MMT6 = drive.trajectoryBuilder(RedP1MMT5.end())
//                    .lineToLinearHeading(new Pose2d(46, -10, Math.toRadians(180)))
//                    .build();
//
//            Trajectory RedP1MMT7 = drive.trajectoryBuilder(RedP1MMT6.end())
//                    .back(10)
//                    .build();
//
//            drive.followTrajectory(RedP1MMT1);
//            drive.followTrajectory(RedP1MMT2);
//            drive.followTrajectory(RedP1MMT3);
//            drive.followTrajectory(RedP1MMT4);
//
//            robot.AMotorUpDown.setPower(0.9);
//            robot.AMotorUpDown.setTargetPosition(1050);
//            robot.AMotorUpDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            while (opModeIsActive() && Math.abs(robot.AMotorUpDown.getCurrentPosition() - robot.AMotorUpDown.getTargetPosition()) > 5) {
//            }
//            robot.AMotorUpDown.setPower(0);
//
//            robot.ClawRotationServo.setPosition(0.2);
//
//            robot.AMotorOutIn.setPower(0.9);
//            robot.AMotorOutIn.setTargetPosition(-1250);
//            robot.AMotorOutIn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            while (opModeIsActive() && Math.abs(robot.AMotorOutIn.getCurrentPosition() - robot.AMotorOutIn.getTargetPosition()) > 5) {
//            }
//            robot.AMotorOutIn.setPower(0);
//
//            robot.ClawDropServo.setPosition(0.785);
//            sleep(200);
//
//            drive.followTrajectory(RedP1MMT5);
//
//            robot.AMotorOutIn.setPower(0.9);
//            robot.AMotorOutIn.setTargetPosition(-760);
//            robot.AMotorOutIn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            while (opModeIsActive() && Math.abs(robot.AMotorOutIn.getCurrentPosition() - robot.AMotorOutIn.getTargetPosition()) > 5) {
//            }
//            robot.AMotorOutIn.setPower(0);
//
//            drive.followTrajectory(RedP1MMT6);
//            drive.followTrajectory(RedP1MMT7);
//
//            robot.ClawRotationServo.setPosition(.36);
//
//            robot.AMotorUpDown.setPower(0.9);
//            robot.AMotorUpDown.setTargetPosition(-300);
//            robot.AMotorUpDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            while (opModeIsActive() && Math.abs(robot.AMotorUpDown.getCurrentPosition() - robot.AMotorUpDown.getTargetPosition()) > 5) {
//            }
//            robot.AMotorUpDown.setPower(0);
//
//
//        } else if (GPTCamera.leftSide == true) {
//            //Mark left
//
//            Trajectory RedP1MLT1 = drive.trajectoryBuilder(RedP1)
//                    .lineToLinearHeading(new Pose2d(30, -44, Math.toRadians(140)))
//                    .build();
//
//            Trajectory RedP1MLT2 = drive.trajectoryBuilder(RedP1MLT1.end())
//                    .forward(12)
//                    .build();
//
//            Trajectory RedP1MLT3 = drive.trajectoryBuilder(RedP1MLT2.end())
//                    .back(12)
//                    .build();
//
//            Trajectory RedP1MLT4 = drive.trajectoryBuilder(RedP1MLT3.end())
//                    .lineToLinearHeading(new Pose2d(46, -35, Math.toRadians(0)))
//                    .build();
//
//            Trajectory RedP1MLT5 = drive.trajectoryBuilder(RedP1MLT4.end())
//                    .back(10)
//                    .build();
//
//            Trajectory RedP1MLT6 = drive.trajectoryBuilder(RedP1MLT5.end())
//                    .lineToLinearHeading(new Pose2d(46, -10, Math.toRadians(180)))
//                    .build();
//
//            Trajectory RedP1MLT7 = drive.trajectoryBuilder(RedP1MLT6.end())
//                    .back(10)
//                    .build();
//
//            drive.followTrajectory(RedP1MLT1);
//            drive.followTrajectory(RedP1MLT2);
//            drive.followTrajectory(RedP1MLT3);
//            drive.followTrajectory(RedP1MLT4);
//
//            robot.AMotorUpDown.setPower(0.9);
//            robot.AMotorUpDown.setTargetPosition(1050);
//            robot.AMotorUpDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            while (opModeIsActive() && Math.abs(robot.AMotorUpDown.getCurrentPosition() - robot.AMotorUpDown.getTargetPosition()) > 5) {
//            }
//            robot.AMotorUpDown.setPower(0);
//
//            robot.ClawRotationServo.setPosition(0.2);
//
//            robot.AMotorOutIn.setPower(0.9);
//            robot.AMotorOutIn.setTargetPosition(-1270);
//            robot.AMotorOutIn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            while (opModeIsActive() && Math.abs(robot.AMotorOutIn.getCurrentPosition() - robot.AMotorOutIn.getTargetPosition()) > 5) {
//            }
//            robot.AMotorOutIn.setPower(0);
//
//            robot.ClawDropServo.setPosition(0.785);
//            sleep(200);
//
//            drive.followTrajectory(RedP1MLT5);
//
//            robot.AMotorOutIn.setPower(0.9);
//            robot.AMotorOutIn.setTargetPosition(-760);
//            robot.AMotorOutIn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            while (opModeIsActive() && Math.abs(robot.AMotorOutIn.getCurrentPosition() - robot.AMotorOutIn.getTargetPosition()) > 5) {
//            }
//            robot.AMotorOutIn.setPower(0);
//
//            drive.followTrajectory(RedP1MLT6);
//            drive.followTrajectory(RedP1MLT7);
//
//            robot.ClawRotationServo.setPosition(.38);
//
//            robot.AMotorUpDown.setPower(0.9);
//            robot.AMotorUpDown.setTargetPosition(-300);
//            robot.AMotorUpDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            while (opModeIsActive() && Math.abs(robot.AMotorUpDown.getCurrentPosition() - robot.AMotorUpDown.getTargetPosition()) > 5) {
//            }
//            robot.AMotorUpDown.setPower(0);
//
//        } else {
//            telemetry.addData("Wait", "Wait");
//            telemetry.update();
//        }
//    }
//}