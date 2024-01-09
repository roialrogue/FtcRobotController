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
        /*Pose2d BlueP1 = new Pose2d(10, 62, Math.toRadians(270));
        drive.setPoseEstimate(BlueP1);*/
        //Path Blue mark 1
        /*Trajectory BlueP1M1T1 = drive.trajectoryBuilder(BlueP1)
                .lineToLinearHeading(new Pose2d(11,32, Math.toRadians(180)))
                .build();

        Trajectory BlueP1M1T2 = drive.trajectoryBuilder(BlueP1M1T1.end())
                .lineToLinearHeading(new Pose2d(50,42, Math.toRadians(0)))
                .build();

        drive.followTrajectory(BlueP1M1T1);
        runtime.reset();
        while (runtime.seconds() < 2) {
            robot.InTakeServo1.setPosition(-1);
            robot.InTakeServo2.setPosition(-1);
        }
        drive.followTrajectory(BlueP1M1T2);*/

        //Path Blue mark 2
        /*Trajectory BlueP1M2T1 = drive.trajectoryBuilder(BlueP1)
                .lineToLinearHeading(new Pose2d(16,34, Math.toRadians(270)))
                .build();

        Trajectory BlueP1M2T2 = drive.trajectoryBuilder(BlueP1M2T1.end())
                .lineToLinearHeading(new Pose2d(50,36, Math.toRadians(0)))
                .build();

        drive.followTrajectory(BlueP1M2T1);
        runtime.reset();
        while (runtime.seconds() < 2) {
            robot.InTakeServo1.setPosition(-1);
            robot.InTakeServo2.setPosition(-1);
        }
        drive.followTrajectory(BlueP1M2T2);*/

        //Path Blue mark 3
        /*Trajectory BlueP1M3T1 = drive.trajectoryBuilder(BlueP1)
                .lineToLinearHeading(new Pose2d(30,42, Math.toRadians(225)))
                .build();

        Trajectory BlueP1M3T2 = drive.trajectoryBuilder(BlueP1M3T1.end())
                .lineToLinearHeading(new Pose2d(50,42, Math.toRadians(0)))
                .build();

        drive.followTrajectory(BlueP1M3T1);
        runtime.reset();
        while (runtime.seconds() < 2) {
            robot.InTakeServo1.setPosition(-1);
            robot.InTakeServo2.setPosition(-1);
        }
        drive.followTrajectory(BlueP1M3T2);*/

        //P.1 Red
        //Starting Position
        Pose2d RedP1 = new Pose2d(14, -62, Math.toRadians(90));
        drive.setPoseEstimate(RedP1);
        //Path Red mark 1
        Trajectory RedP1M1T1 = drive.trajectoryBuilder(RedP1)
                .lineToLinearHeading(new Pose2d(12,-32, Math.toRadians(180)))
                .build();

        Trajectory RedP1M1T2 = drive.trajectoryBuilder(RedP1M1T1.end())
                .lineToLinearHeading(new Pose2d(50,-30, Math.toRadians(0)))
                .build();
        //in init

        robot.ClawRotationServo.setPosition(.333);
        robot.ClawDropServo.setPosition(.5);

        robot.AMotorOutIn.setPower(0.7);
        robot.AMotorOutIn.setTargetPosition(-300);
        robot.AMotorOutIn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(opModeIsActive() && robot.AMotorOutIn.isBusy()) {}
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
        while(opModeIsActive() && robot.AMotorUpDown.isBusy()) {}
        robot.AMotorUpDown.setPower(0);

        robot.ClawRotationServo.setPosition(.23);
        robot.AMotorOutIn.setPower(0.7);
        robot.AMotorOutIn.setTargetPosition(-1500);
        robot.AMotorOutIn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(opModeIsActive() && robot.AMotorOutIn.isBusy()) {}
        robot.AMotorOutIn.setPower(0);

        robot.ClawDropServo.setPosition(0.040);
        robot.ClawDropServo.setPosition(0.095);



        //Path Red mark 2
        /*Trajectory RedP1M2T1 = drive.trajectoryBuilder(RedP1)
                .lineToLinearHeading(new Pose2d(16,-34, Math.toRadians(90)))
                .build();

        Trajectory RedP1M2T2 = drive.trajectoryBuilder(RedP1M2T1.end())
                .lineToLinearHeading(new Pose2d(50,-36, Math.toRadians(0)))
                .build();

        drive.followTrajectory(RedP1M2T1);
        runtime.reset();
        while (runtime.seconds() < 2) {
            robot.InTakeServo1.setPosition(-1);
            robot.InTakeServo2.setPosition(-1);
        }
        drive.followTrajectory(RedP1M2T2);*/

        //Path Red mark 3
        /*Trajectory RedP1M3T1 = drive.trajectoryBuilder(RedP1)
                .lineToLinearHeading(new Pose2d(30,-44, Math.toRadians(125)))
                .build();

        Trajectory RedP1M3T2 = drive.trajectoryBuilder(RedP1M3T1.end())
                .lineToLinearHeading(new Pose2d(50,-42, Math.toRadians(0)))
                .build();

        drive.followTrajectory(RedP1M3T1);
        runtime.reset();
        while (runtime.seconds() < 2) {
            robot.InTakeServo1.setPosition(-1);
            robot.InTakeServo2.setPosition(-1);
        }
        drive.followTrajectory(RedP1M3T2);*/
/*

        //P.2 Blue
        //Starting Position
        Pose2d BlueP2 = new Pose2d(-38, 62, Math.toRadians(270));
        drive.setPoseEstimate(BlueP2);
        //Path Blue mark 1
        Trajectory BlueP2M1T1 = drive.trajectoryBuilder(BlueP2)
                .lineToLinearHeading(new Pose2d(-34,32, Math.toRadians(0)))
                .build();

        Trajectory BlueP2M1T2 = drive.trajectoryBuilder(BlueP2M1T1.end())
                .lineToLinearHeading(new Pose2d(-59,36, Math.toRadians(180)))
                .build();

        Trajectory BlueP2M1T3 = drive.trajectoryBuilder(BlueP2M1T2.end())
                .lineToLinearHeading(new Pose2d(-33,58, Math.toRadians(0)))
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

        //Path Blue mark 2
        Trajectory BlueP2M2T1 = drive.trajectoryBuilder(BlueP2)
                .lineToLinearHeading(new Pose2d(-34,34, Math.toRadians(270)))
                .build();

        Trajectory BlueP2M2T2 = drive.trajectoryBuilder(BlueP2M2T1.end())
                .lineToLinearHeading(new Pose2d(-59,36, Math.toRadians(180)))
                .build();

        Trajectory BlueP2M2T3 = drive.trajectoryBuilder(BlueP2M2T2.end())
                .lineToLinearHeading(new Pose2d(-33,58, Math.toRadians(0)))
                .build();

        drive.followTrajectory(BlueP2M2T1);
        runtime.reset();
        while (runtime.seconds() < 2) {
            robot.InTakeServo1.setPosition(-1);
            robot.InTakeServo2.setPosition(-1);
        }
        drive.followTrajectory(BlueP2M2T2);
        while (runtime.seconds() < 2) {
            robot.InTakeServo1.setPosition(1);
            robot.InTakeServo2.setPosition(1);
        }
        drive.followTrajectory(BlueP2M2T3);

        //Path Blue mark 3
        Trajectory BlueP2M3T1 = drive.trajectoryBuilder(BlueP2)
                .lineToLinearHeading(new Pose2d(0,0, Math.toRadians(0)))
                .build();

        Trajectory BlueP2M3T2 = drive.trajectoryBuilder(BlueP2M3T1.end())
                .lineToLinearHeading(new Pose2d(0,0, Math.toRadians(0)))
                .build();

        Trajectory BlueP2M3T3 = drive.trajectoryBuilder(BlueP2M3T2.end())
                .lineToLinearHeading(new Pose2d(0,0, Math.toRadians(0)))
                .build();

        drive.followTrajectory(BlueP2M3T1);
        runtime.reset();
        while (runtime.seconds() < 2) {
            robot.InTakeServo1.setPosition(-1);
            robot.InTakeServo2.setPosition(-1);
        }
        drive.followTrajectory(BlueP2M3T2);
        while (runtime.seconds() < 2) {
            robot.InTakeServo1.setPosition(1);
            robot.InTakeServo2.setPosition(1);
        }
        drive.followTrajectory(BlueP2M3T3);

        //P.2 Red
        //Starting Position
        Pose2d RedP2 = new Pose2d(-33, -63, Math.toRadians(90));
        drive.setPoseEstimate(RedP2);
        //Path Red mark 1
        Trajectory RedP2M1T1 = drive.trajectoryBuilder(RedP2)
                .lineToLinearHeading(new Pose2d(-34,-32, Math.toRadians(0)))
                .build();

        Trajectory RedP2M1T2 = drive.trajectoryBuilder(RedP2M1T1.end())
                .lineToLinearHeading(new Pose2d(-59,-36, Math.toRadians(180)))
                .build();

        Trajectory RedP2M1T3 = drive.trajectoryBuilder(RedP2M1T2.end())
                .lineToLinearHeading(new Pose2d(-33,-58, Math.toRadians(0)))
                .build();

        drive.followTrajectory(RedP2M1T1);
        runtime.reset();
        while (runtime.seconds() < 2) {
            robot.InTakeServo1.setPosition(-1);
            robot.InTakeServo2.setPosition(-1);
        }
        drive.followTrajectory(RedP2M1T2);
        while (runtime.seconds() < 2) {
            robot.InTakeServo1.setPosition(1);
            robot.InTakeServo2.setPosition(1);
        }
        drive.followTrajectory(RedP2M1T3);

        //Path Red mark 2
        Trajectory RedP2M2T1 = drive.trajectoryBuilder(RedP2)
                .lineToLinearHeading(new Pose2d(-34,-34, Math.toRadians(90)))
                .build();

        Trajectory RedP2M2T2 = drive.trajectoryBuilder(RedP2M2T1.end())
                .lineToLinearHeading(new Pose2d(-59,-36, Math.toRadians(180)))
                .build();

        Trajectory RedP2M2T3 = drive.trajectoryBuilder(RedP2M2T2.end())
                .lineToLinearHeading(new Pose2d(-33,-58, Math.toRadians(0)))
                .build();

        drive.followTrajectory(RedP2M2T1);
        runtime.reset();
        while (runtime.seconds() < 2) {
            robot.InTakeServo1.setPosition(-1);
            robot.InTakeServo2.setPosition(-1);
        }
        drive.followTrajectory(RedP2M2T2);
        runtime.reset();
        while (runtime.seconds() < 2) {
            robot.InTakeServo1.setPosition(1);
            robot.InTakeServo2.setPosition(1);
        }
        drive.followTrajectory(RedP2M2T3);

        //Path Red mark 3
        Trajectory RedP2M3T1 = drive.trajectoryBuilder(RedP2)
                .lineToLinearHeading(new Pose2d(0,0, Math.toRadians(0)))
                .build();

        Trajectory RedP2M3T2 = drive.trajectoryBuilder(RedP2M3T1.end())
                .lineToLinearHeading(new Pose2d(0,0, Math.toRadians(0)))
                .build();

        Trajectory RedP2M3T3 = drive.trajectoryBuilder(RedP2M3T2.end())
                .lineToLinearHeading(new Pose2d(0,0, Math.toRadians(0)))
                .build();

        drive.followTrajectory(RedP2M3T1);
        runtime.reset();
        while (runtime.seconds() < 2) {
            robot.InTakeServo1.setPosition(-1);
            robot.InTakeServo2.setPosition(-1);
        }
        drive.followTrajectory(RedP2M3T2);
        while (runtime.seconds() < 2) {
            robot.InTakeServo1.setPosition(1);
            robot.InTakeServo2.setPosition(1);
        }
        drive.followTrajectory(RedP2M3T3);
*/




        /*drive.trajectorySequenceBuilder(new Pose2d(-53, -38, Math.toRadians(170)))
                .lineToLinearHeading(new Pose2d(-33,-60, Math.toRadians(0)))
                .forward(30)
                .splineToConstantHeading(new Vector2d(52, -29), Math.toRadians(0));*/

        /*Trajectory trajP2Mark3T1 = drive.trajectoryBuilder(BlueP1)
                .lineToLinearHeading(new Pose2d(-43,-42, Math.toRadians(90)))
                .build();


        Trajectory trajP2Mark3T2 = drive.trajectoryBuilder(trajP2Mark3T1.end())
                .lineToLinearHeading(new Pose2d(-55,-40, Math.toRadians(180)))
                .build();

        Trajectory trajP2Mark3T3 = drive.trajectoryBuilder(trajP2Mark3T2.end())
                .splineTo(new Vector2d(0, -60), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(52, -29), Math.toRadians(0))
                .build();*/


                /*Trajectory trajP2Mark3T3 = drive.trajectoryBuilder(trajP2Mark3T2.end(),true)
                .splineToConstantHeading(new Vector2d(52, -29), Math.toRadians(0))
                .build();*/

        /*Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .splineTo(new Vector2d(20, 9), Math.toRadians(45))
                .build();*/

        /*drive.followTrajectory(trajP2Mark3T1);
        runtime.reset();
        while (runtime.seconds() < 2) {
            robot.InTakeServo1.setPosition(-1);
            robot.InTakeServo2.setPosition(-1);
        }
        *//*drive.turn(Math.toRadians(125));*//*
        drive.followTrajectory(trajP2Mark3T2);
        while (runtime.seconds() < 3) {
            robot.InTakeServo1.setPosition(1);
            robot.InTakeServo2.setPosition(1);
        }*/
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