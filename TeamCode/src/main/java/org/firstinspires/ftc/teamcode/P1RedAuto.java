package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

        waitForStart();

        Pose2d RedP2Mark3 = new Pose2d(-33, -63, Math.toRadians(90));

        drive.setPoseEstimate(RedP2Mark3);

        /*drive.trajectorySequenceBuilder(new Pose2d(-53, -38, Math.toRadians(170)))
                .lineToLinearHeading(new Pose2d(-33,-60, Math.toRadians(0)))
                .forward(30)
                .splineToConstantHeading(new Vector2d(52, -29), Math.toRadians(0));*/

        Trajectory trajP2Mark3T1 = drive.trajectoryBuilder(RedP2Mark3)
                .lineToLinearHeading(new Pose2d(-43,-42, Math.toRadians(90)))
                .build();


        Trajectory trajP2Mark3T2 = drive.trajectoryBuilder(trajP2Mark3T1.end())
                .lineToLinearHeading(new Pose2d(-55,-40, Math.toRadians(180)))
                .build();

        Trajectory trajP2Mark3T3 = drive.trajectoryBuilder(trajP2Mark3T2.end())
                .splineTo(new Vector2d(0, -60), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(52, -29), Math.toRadians(0))
                .build();


                /*Trajectory trajP2Mark3T3 = drive.trajectoryBuilder(trajP2Mark3T2.end(),true)
                .splineToConstantHeading(new Vector2d(52, -29), Math.toRadians(0))
                .build();*/

        /*Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .splineTo(new Vector2d(20, 9), Math.toRadians(45))
                .build();*/

        drive.followTrajectory(trajP2Mark3T1);
        runtime.reset();
        while (runtime.seconds() < 2) {
            robot.InTakeServo1.setPosition(-1);
            robot.InTakeServo2.setPosition(-1);
        }
        /*drive.turn(Math.toRadians(125));*/
        drive.followTrajectory(trajP2Mark3T2);
        while (runtime.seconds() < 3) {
            robot.InTakeServo1.setPosition(1);
            robot.InTakeServo2.setPosition(1);
        }
        /*drive.followTrajectory(trajP2Mark3T3);*/


        //Blue P.2 mark 3 to the line
        //will need to be edit depending on exact start
        /*Pose2d BluePP = new Pose2d(-37, 63, Math.toRadians(0));

        Trajectory BlueP2Mark3 = drive.trajectoryBuilder(BluePP)
                .lineToLinearHeading(new Pose2d(-52,38, Math.toRadians(300)))
                .build();
        drive.followTrajectory(BlueP2Mark3);
        drive.turn(Math.toRadians(-115));*/
    }
}