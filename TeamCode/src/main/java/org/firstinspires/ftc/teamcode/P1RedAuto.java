package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import android.net.Uri;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "P.1 Red Auto")
public class P1RedAuto extends LinearOpMode{
    boolean isBlue;
    Hardware robot = Hardware.getInstance();
    ElapsedTime runtime = new ElapsedTime();


    public void runOpMode() {
        robot.init(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        // We want to start the bot at x: 10, y: -8, heading: 90 degrees
        Pose2d startPose = new Pose2d(-72, 36, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        Trajectory myTrajectory = drive.trajectoryBuilder(startPose)
                .forward(25)
                .build();
        Trajectory myTrajectory2 = drive.trajectoryBuilder(myTrajectory.end())
                .strafeRight(72)
                .build();

        /*Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(, 9), Math.toRadians(45))
                .build();
*/
        /*Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .splineTo(new Vector2d(20, 9), Math.toRadians(45))
                .build();*/

        drive.followTrajectory(myTrajectory);
        drive.followTrajectory(myTrajectory2);
    }


    }