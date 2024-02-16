package org.firstinspires.ftc.teamcode.Pipelines;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;

@Autonomous(name = "Turning")
public class TestRoadRunnerHeading extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime();
    Hardware robot = Hardware.getInstance();
    OpenCvCamera webCam;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    myGamePad myGamepad = new myGamePad(gamepad1);

    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d BlueP1 = new Pose2d(0, 0, Math.toRadians(180));
        drive.setPoseEstimate(BlueP1);

        Trajectory BlueP1MRT1 = drive.trajectoryBuilder(BlueP1)
                .lineToLinearHeading(new Pose2d(10, 0, Math.toRadians(360)))
                .build();

        Trajectory BlueP1MRT2 = drive.trajectoryBuilder(BlueP1MRT1.end())
                .lineToLinearHeading(new Pose2d(0, 0, Math.toRadians(180)))
                .build();

        Trajectory BlueP1MRT3 = drive.trajectoryBuilder(BlueP1MRT2.end())
                .lineToLinearHeading(new Pose2d(10, 0, Math.toRadians(360)))
                .build();

        Trajectory BlueP1MRT4 = drive.trajectoryBuilder(BlueP1MRT3.end())
                .lineToLinearHeading(new Pose2d(10, 0, Math.toRadians(180)))
                .build();

        Trajectory BlueP1MRT5 = drive.trajectoryBuilder(BlueP1MRT4.end())
                .lineToLinearHeading(new Pose2d(0, 0, Math.toRadians(360)))
                .build();

        Trajectory BlueP1MRT6 = drive.trajectoryBuilder(BlueP1MRT5.end())
                .lineToLinearHeading(new Pose2d(10, 0, Math.toRadians(180)))
                .build();

        Trajectory BlueP1MRT7 = drive.trajectoryBuilder(BlueP1MRT6.end())
                .lineToLinearHeading(new Pose2d(0, 0, Math.toRadians(360)))
                .build();

        Trajectory BlueP1MRT8 = drive.trajectoryBuilder(BlueP1MRT7.end())
                .lineToLinearHeading(new Pose2d(10, 0, Math.toRadians(180)))
                .build();

        Trajectory BlueP1MRT9 = drive.trajectoryBuilder(BlueP1MRT8.end())
                .lineToLinearHeading(new Pose2d(0, 0, Math.toRadians(360)))
                .build();

        Trajectory BlueP1MRT10 = drive.trajectoryBuilder(BlueP1MRT9.end())
                .lineToLinearHeading(new Pose2d(10, 0, Math.toRadians(180)))
                .build();

        drive.followTrajectory(BlueP1MRT1);
        drive.followTrajectory(BlueP1MRT2);
        drive.followTrajectory(BlueP1MRT3);
        drive.followTrajectory(BlueP1MRT4);
        drive.followTrajectory(BlueP1MRT5);
        drive.followTrajectory(BlueP1MRT6);
        drive.followTrajectory(BlueP1MRT7);
        drive.followTrajectory(BlueP1MRT8);
        drive.followTrajectory(BlueP1MRT9);
        drive.followTrajectory(BlueP1MRT10);


    }
}