package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.opencv.core.Point;

@Autonomous(name = "P.1 Red Auto")
public class P1RedAuto extends LinearOpMode {
    boolean isBlue;
    Hardware robot = Hardware.getInstance();
    ElapsedTime runtime = new ElapsedTime();


    public void runOpMode() {
        robot.init(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        robot.AMotorUpDown.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.AMotorOutIn.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.AMotorUpDown.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.AMotorOutIn.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();

        telemetry.addData("AMoterUpDown position", robot.AMotorUpDown.getCurrentPosition());
        telemetry.addData("AMoterOutIn position", robot.AMotorOutIn.getCurrentPosition());
        telemetry.update();

        //P.1 Blue
        //Starting Position
        Pose2d BlueP1 = new Pose2d(10, 62, Math.toRadians(270));
        drive.setPoseEstimate(BlueP1);
        //Path Blue mark 1
        Trajectory BlueP1M1T1 = drive.trajectoryBuilder(BlueP1)
                .lineToLinearHeading(new Pose2d(11, 32, Math.toRadians(180)))
                .build();

        Trajectory BlueP1M1T2 = drive.trajectoryBuilder(BlueP1M1T1.end())
                .lineToLinearHeading(new Pose2d(50, 42, Math.toRadians(0)))
                .build();

        Trajectory BlueP1M1T3 = drive.trajectoryBuilder(BlueP1M1T2.end())
                .strafeTo(new Vector2d(50, 60))
                .build();

        drive.followTrajectory(BlueP1M1T1);
        runtime.reset();
        while (runtime.seconds() < 2) {
            robot.InTakeServo1.setPosition(-1);
            robot.InTakeServo2.setPosition(-1);
        }
        drive.followTrajectory(BlueP1M1T2);
        drive.followTrajectory(BlueP1M1T3);
        drive.turn(180);

        //Path Blue mark 2
        Trajectory BlueP1M2T1 = drive.trajectoryBuilder(BlueP1)
                .lineToLinearHeading(new Pose2d(16, 34, Math.toRadians(270)))
                .build();

        Trajectory BlueP1M2T2 = drive.trajectoryBuilder(BlueP1M2T1.end())
                .lineToLinearHeading(new Pose2d(50, 36, Math.toRadians(0)))
                .build();

        Trajectory BlueP1M2T3 = drive.trajectoryBuilder(BlueP1M2T2.end())
                .strafeTo(new Vector2d(50, 60))
                .build();

        drive.followTrajectory(BlueP1M2T1);
        runtime.reset();
        while (runtime.seconds() < 2) {
            robot.InTakeServo1.setPosition(-1);
            robot.InTakeServo2.setPosition(-1);
        }
        drive.followTrajectory(BlueP1M2T2);
        drive.followTrajectory(BlueP1M2T3);
        drive.turn(180);

        //Path Blue mark 3
        Trajectory BlueP1M3T1 = drive.trajectoryBuilder(BlueP1)
                .lineToLinearHeading(new Pose2d(30, 42, Math.toRadians(225)))
                .build();

        Trajectory BlueP1M3T2 = drive.trajectoryBuilder(BlueP1M3T1.end())
                .lineToLinearHeading(new Pose2d(50, 42, Math.toRadians(0)))
                .build();

        Trajectory BlueP1M3T3 = drive.trajectoryBuilder(BlueP1M3T2.end())
                .strafeTo(new Vector2d(50, 60))
                .build();

        drive.followTrajectory(BlueP1M3T1);
        runtime.reset();
        while (runtime.seconds() < 2) {
            robot.InTakeServo1.setPosition(-1);
            robot.InTakeServo2.setPosition(-1);
        }
        drive.followTrajectory(BlueP1M3T2);
        drive.followTrajectory(BlueP1M3T3);
        drive.turn(180);

        //P.1 Red
        //Starting Position
        Pose2d RedP1 = new Pose2d(14, -62, Math.toRadians(90));
        drive.setPoseEstimate(RedP1);
        //Path Red mark 1
        Trajectory RedP1M1T1 = drive.trajectoryBuilder(RedP1)
                .lineToLinearHeading(new Pose2d(12, -32, Math.toRadians(180)))
                .build();

        Trajectory RedP1M1T2 = drive.trajectoryBuilder(RedP1M1T1.end())
                .lineToLinearHeading(new Pose2d(50, -30, Math.toRadians(0)))
                .build();

        Trajectory RedP1M1T3 = drive.trajectoryBuilder(RedP1M1T2.end())
                .strafeTo(new Vector2d(50, -60))
                .build();
        //in init

        robot.ClawRotationServo.setPosition(.333);
        robot.ClawDropServo.setPosition(.5);

        robot.AMotorOutIn.setPower(0.7);
        robot.AMotorOutIn.setTargetPosition(-300);
        robot.AMotorOutIn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (opModeIsActive() && robot.AMotorOutIn.isBusy()) {
        }
        robot.AMotorOutIn.setPower(0);
        drive.followTrajectory(RedP1M1T1);

        runtime.reset();
        while (runtime.seconds() < 2) {
            robot.InTakeServo1.setPosition(-1);
            robot.InTakeServo2.setPosition(-1);
        }
        robot.InTakeServo1.setPosition(.5);
        robot.InTakeServo2.setPosition(.5);
        drive.followTrajectory(RedP1M1T2);

        robot.AMotorUpDown.setPower(0.7);
        robot.AMotorUpDown.setTargetPosition(1000);
        robot.AMotorUpDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (opModeIsActive() && robot.AMotorUpDown.isBusy()) {
        }
        robot.AMotorUpDown.setPower(0);

        robot.ClawRotationServo.setPosition(.23);
        robot.AMotorOutIn.setPower(0.7);
        robot.AMotorOutIn.setTargetPosition(-1500);
        robot.AMotorOutIn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (opModeIsActive() && robot.AMotorOutIn.isBusy()) {
        }
        robot.AMotorOutIn.setPower(0);

        robot.ClawDropServo.setPosition(0.040);
        robot.ClawDropServo.setPosition(0.095);
        drive.followTrajectory(RedP1M1T3);
        drive.turn(180);


        //Path Red mark 2
        Trajectory RedP1M2T1 = drive.trajectoryBuilder(RedP1)
                .lineToLinearHeading(new Pose2d(16, -34, Math.toRadians(90)))
                .build();

        Trajectory RedP1M2T2 = drive.trajectoryBuilder(RedP1M2T1.end())
                .lineToLinearHeading(new Pose2d(50, -36, Math.toRadians(0)))
                .build();

        Trajectory RedP1M2T3 = drive.trajectoryBuilder(RedP1M2T2.end())
                .strafeTo(new Vector2d(50, -60))
                .build();

        drive.followTrajectory(RedP1M2T1);
        runtime.reset();
        while (runtime.seconds() < 2) {
            robot.InTakeServo1.setPosition(-1);
            robot.InTakeServo2.setPosition(-1);
        }
        drive.followTrajectory(RedP1M2T2);
        drive.followTrajectory(RedP1M2T3);
        drive.turn(180);

        //Path Red mark 3
        Trajectory RedP1M3T1 = drive.trajectoryBuilder(RedP1)
                .lineToLinearHeading(new Pose2d(30, -44, Math.toRadians(125)))
                .build();

        Trajectory RedP1M3T2 = drive.trajectoryBuilder(RedP1M3T1.end())
                .lineToLinearHeading(new Pose2d(50, -42, Math.toRadians(0)))
                .build();

        Trajectory RedP1M3T3 = drive.trajectoryBuilder(RedP1M3T2.end())
                .strafeTo(new Vector2d(50, -60))
                .build();

        drive.followTrajectory(RedP1M3T1);
        runtime.reset();
        while (runtime.seconds() < 2) {
            robot.InTakeServo1.setPosition(-1);
            robot.InTakeServo2.setPosition(-1);
        }
        drive.followTrajectory(RedP1M3T2);
        drive.followTrajectory(RedP1M3T3);
        drive.turn(180);

        //P.2 Blue
        //Starting Position
        Pose2d BlueP2 = new Pose2d(-38, 62, Math.toRadians(270));
        drive.setPoseEstimate(BlueP2);
        //Path Blue mark 1
        Trajectory BlueP2M1T1 = drive.trajectoryBuilder(BlueP2)
                .lineToLinearHeading(new Pose2d(-34, 32, Math.toRadians(0)))
                .build();

        Trajectory BlueP2M1T2 = drive.trajectoryBuilder(BlueP2M1T1.end())
                .lineToLinearHeading(new Pose2d(-33, 58, Math.toRadians(0)))
                .build();

        Trajectory BlueP2M1T3 = drive.trajectoryBuilder(BlueP2M1T2.end())
                .lineToLinearHeading(new Pose2d(5, 58, Math.toRadians(0)))
                .build();

        Trajectory BlueP2M1T4 = drive.trajectoryBuilder(BlueP2M1T3.end())
                .splineToConstantHeading(new Vector2d(52, 42), Math.toRadians(0))
                .build();

        Trajectory BlueP2M1T5 = drive.trajectoryBuilder(BlueP2M1T4.end())
                .strafeTo(new Vector2d(50, 60))
                .build();

        drive.followTrajectory(BlueP2M1T1);
        runtime.reset();
        while (runtime.seconds() < 2) {
            robot.InTakeServo1.setPosition(-1);
            robot.InTakeServo2.setPosition(-1);
        }
        drive.followTrajectory(BlueP2M1T2);
        while (runtime.seconds() < 2) {
            robot.InTakeServo1.setPosition(1);
            robot.InTakeServo2.setPosition(1);
        }
        drive.followTrajectory(BlueP2M1T3);
        drive.followTrajectory(BlueP2M1T4);
        drive.followTrajectory(BlueP2M1T5);
        drive.turn(180);

        //Path Blue mark 2
        Trajectory BlueP2M2T1 = drive.trajectoryBuilder(BlueP2)
                .lineToLinearHeading(new Pose2d(-34, 34, Math.toRadians(270)))
                .build();

        Trajectory BlueP2M2T2 = drive.trajectoryBuilder(BlueP2M2T1.end())
                .lineToLinearHeading(new Pose2d(-35, 58, Math.toRadians(0)))
                .build();

        Trajectory BlueP2M2T3 = drive.trajectoryBuilder(BlueP2M2T2.end())
                .splineToConstantHeading(new Vector2d(52, 42), Math.toRadians(0))
                .build();

        Trajectory BlueP2M2T4 = drive.trajectoryBuilder(BlueP2M2T3.end())
                .lineToLinearHeading(new Pose2d(5, 58, Math.toRadians(0)))
                .build();

        Trajectory BlueP2M2T5 = drive.trajectoryBuilder(BlueP2M2T4.end())
                .strafeTo(new Vector2d(50, 60))
                .build();

        drive.followTrajectory(BlueP2M2T1);
        runtime.reset();
        while (runtime.seconds() < 2) {
            robot.InTakeServo1.setPosition(-1);
            robot.InTakeServo2.setPosition(-1);
        }
        drive.followTrajectory(BlueP2M2T2);
        drive.followTrajectory(BlueP2M2T3);
        drive.followTrajectory(BlueP2M2T4);
        drive.followTrajectory(BlueP2M2T5);
        drive.turn(180);

        //Path Blue mark 3
        Trajectory BlueP2M3T1 = drive.trajectoryBuilder(BlueP2)
                .lineToLinearHeading(new Pose2d(-34, 32, Math.toRadians(180)))
                .build();

        Trajectory BlueP2M3T2 = drive.trajectoryBuilder(BlueP2M3T1.end())
                .lineToLinearHeading(new Pose2d(-38, 58, Math.toRadians(0)))
                .build();

        Trajectory BlueP2M3T3 = drive.trajectoryBuilder(BlueP2M3T2.end())
                .lineToLinearHeading(new Pose2d(5, 58, Math.toRadians(0)))
                .build();

        Trajectory BlueP2M3T4 = drive.trajectoryBuilder(BlueP2M3T3.end())
                .splineToConstantHeading(new Vector2d(52, 42), Math.toRadians(0))
                .build();

        Trajectory BlueP2M3T5 = drive.trajectoryBuilder(BlueP2M3T4.end())
                .strafeTo(new Vector2d(50, 60))
                .build();

        drive.followTrajectory(BlueP2M3T1);
        runtime.reset();
        while (runtime.seconds() < 2) {
            robot.InTakeServo1.setPosition(-1);
            robot.InTakeServo2.setPosition(-1);
        }
        drive.followTrajectory(BlueP2M3T2);
        drive.followTrajectory(BlueP2M3T3);
        drive.followTrajectory(BlueP2M3T4);
        drive.followTrajectory(BlueP2M3T5);
        drive.turn(180);

        //P.2 Red
        //Starting Position
        Pose2d RedP2 = new Pose2d(-33, -63, Math.toRadians(90));
        drive.setPoseEstimate(RedP2);
        //Path Red mark 1
        Trajectory RedP2M1T1 = drive.trajectoryBuilder(RedP2)
                .lineToLinearHeading(new Pose2d(-34, -32, Math.toRadians(0)))
                .build();

        Trajectory RedP2M1T2 = drive.trajectoryBuilder(RedP2M1T1.end())
                .lineToLinearHeading(new Pose2d(-33, -58, Math.toRadians(0)))
                .build();

        Trajectory RedP2M1T3 = drive.trajectoryBuilder(RedP2M1T2.end())
                .lineToLinearHeading(new Pose2d(5, -58, Math.toRadians(0)))
                .build();

        Trajectory RedP2M1T4 = drive.trajectoryBuilder(RedP2M1T3.end())
                .splineToConstantHeading(new Vector2d(50, -42), Math.toRadians(0))
                .build();

        Trajectory RedP2M1T5 = drive.trajectoryBuilder(RedP2M1T4.end())
                .strafeTo(new Vector2d(50, -60))
                .build();


        drive.followTrajectory(RedP2M1T1);
        runtime.reset();
        while (runtime.seconds() < 2) {
            robot.InTakeServo1.setPosition(-1);
            robot.InTakeServo2.setPosition(-1);
        }
        drive.followTrajectory();
        drive.followTrajectory(RedP2M1T2);
        drive.followTrajectory(RedP2M1T3);
        drive.followTrajectory(RedP2M1T4);
        drive.followTrajectory(RedP2M1T5);
        drive.turn(180);

        //Path Red mark 2
        Trajectory RedP2M2T1 = drive.trajectoryBuilder(RedP2)
                .lineToLinearHeading(new Pose2d(-34, -34, Math.toRadians(90)))
                .build();

        Trajectory RedP2M2T2 = drive.trajectoryBuilder(RedP2M2T1.end())
                .lineToLinearHeading(new Pose2d(-59, -36, Math.toRadians(180)))
                .build();

        Trajectory RedP2M2T3 = drive.trajectoryBuilder(RedP2M2T2.end())
                .lineToLinearHeading(new Pose2d(-33, -58, Math.toRadians(0)))
                .build();

        drive.followTrajectory(RedP2M2T1);
        runtime.reset();
        while (runtime.seconds() < 2) {
            robot.InTakeServo1.setPosition(-1);
            robot.InTakeServo2.setPosition(-1);
        }
        drive.followTrajectory(RedP2M2T2);


        drive.followTrajectory(RedP2M2T3);

        //Path Red mark 3
        Trajectory RedP2M3T1 = drive.trajectoryBuilder(RedP2)
                .lineToLinearHeading(new Pose2d(-34, -32, Math.toRadians(180)))
                .build();

        Trajectory RedP2M3T2 = drive.trajectoryBuilder(RedP2M3T1.end())
                .lineToLinearHeading(new Pose2d(-33, -58, Math.toRadians(0)))
                .build();

        Trajectory RedP2M3T3 = drive.trajectoryBuilder(RedP2M3T2.end())
                .lineToLinearHeading(new Pose2d(5, -58, Math.toRadians(0)))
                .build();

        Trajectory RedP2M3T4 = drive.trajectoryBuilder(RedP2M3T3.end())
                .splineToConstantHeading(new Vector2d(50, -30), Math.toRadians(0))
                .build();

        Trajectory RedP2M3T5 = drive.trajectoryBuilder(RedP2M3T4.end())
                .strafeTo(new Vector2d(50, -60))
                .build();

        drive.followTrajectory(RedP2M3T1);
        runtime.reset();
        while (runtime.seconds() < 2) {
            robot.InTakeServo1.setPosition(-1);
            robot.InTakeServo2.setPosition(-1);
        }
        drive.followTrajectory(RedP2M3T1);
        drive.followTrajectory(RedP2M3T2);

        drive.followTrajectory(RedP2M3T4);
        drive.followTrajectory(RedP2M3T5);
        drive.turn(180);

    }
}