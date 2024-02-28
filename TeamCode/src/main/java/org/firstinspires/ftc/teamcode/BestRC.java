package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Autos.ZP1BlueAuto;
import org.firstinspires.ftc.teamcode.Autos.ZP1RedAuto;
import org.firstinspires.ftc.teamcode.Pipelines.myGamePad;
@Disabled
@TeleOp(name = "BestTeleOp")
public class BestRC extends LinearOpMode {

    Hardware robot = Hardware.getInstance();

    public void runOpMode() {
        robot.init(hardwareMap);

        if (robot.rightForwardWheel != null) {
            robot.rightForwardWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if (robot.leftForwardWheel != null) {
            robot.leftForwardWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if (robot.rightRearWheel != null) {
            robot.rightRearWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if (robot.leftRearWheel != null) {
            robot.leftRearWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        double airplaneDisengaged = 0.3;
        double airplaneEngaged = 0.18;

        double rightOpen = 0.18;
        double rightClosed = 0.50;

        double leftOpen = 0.95;
        double leftClosed = 0.65;

        double grounded = 0.50;
        double board = 0.7;
        double boardInvert = 0.08;

        double flat = 0.725;
        double invert = 0.05;

        robot.RightInTake.setPosition(rightOpen);
        robot.LeftInTake.setPosition(leftOpen);
        robot.ClawLeftRight.setPosition(flat);
        robot.ClawUpDown.setPosition(grounded);
        robot.AirplaneServo.setPosition(airplaneDisengaged);

        robot.BeltMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BeltMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

        waitForStart();

        boolean pressingRB = true;
        boolean pressingLB = true;
        boolean pressedX = true;
        boolean pressedA = true;

        while (opModeIsActive()) {

            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;
            boolean mode = gamepad2.b;
            boolean flip = gamepad2.x;

            double speed = 1;
            if (gamepad1.left_bumper) {
                speed = 0.33;
            }

            double rfm = axial - lateral - yaw;
            double rbm = axial + lateral - yaw;
            double lfm = axial + lateral + yaw;
            double lbm = axial - lateral + yaw;

            double max;
            max = Math.max(Math.abs(lfm), Math.abs(rfm));
            max = Math.max(max, Math.abs(lbm));
            max = Math.max(max, Math.abs(rbm));

            if (max > 1.0) {
                lfm /= max;
                rfm /= max;
                lbm /= max;
                rbm /= max;
            }

            robot.rightForwardWheel.setPower(rfm * speed);
            robot.rightRearWheel.setPower(rbm * speed);
            robot.leftForwardWheel.setPower(lfm * speed);
            robot.leftRearWheel.setPower(lbm * speed);

            double Ticks = robot.BeltMotor.getCurrentPosition();

            if (gamepad2.left_stick_y < -0.1) {
                if (400 < Ticks && Ticks < 2400) {
                    robot.BeltMotor.setPower(1);
                } else if (2400 < Ticks && Ticks < 2600) {
                    robot.BeltMotor.setPower(0.3);
                } else if (2600 < Ticks && Ticks < 2800) {
                    robot.BeltMotor.setPower(0.1);
                } else if (Ticks > 2800) {
                    robot.BeltMotor.setPower(0.01);
                }
            } else if (gamepad2.left_stick_y > 0.1) {
                if (Ticks < 150) {
                    robot.BeltMotor.setPower(-0.15);
                } else if (Ticks < 250) {
                    robot.BeltMotor.setPower(-0.4);
                } else if (Ticks < 400) {
                    robot.BeltMotor.setPower(-0.7);
                } else if (400 < Ticks && Ticks < 2400) {
                    robot.BeltMotor.setPower(-1);
                }
            } else {
                robot.BeltMotor.setPower(0);
            }

            telemetry.addData("?",flip);
            telemetry.update();

            if (Ticks <= 1200) {
                    robot.ClawUpDown.setPosition(grounded);
                    robot.ClawLeftRight.setPosition(flat);
                } else if (Ticks > 1800 && !flip) {
                    robot.ClawLeftRight.setPosition(invert);
                    robot.ClawUpDown.setPosition(boardInvert);
            }


                if (flip) {
                    if (pressedX) {
                        robot.ClawUpDown.setPosition((Math.round(robot.ClawUpDown.getPosition() * 1000) / 1000.0 == Math.round(board * 1000) / 1000.0) ? boardInvert : board);
                        robot.ClawLeftRight.setPosition((Math.round(robot.ClawLeftRight.getPosition() * 1000) / 1000.0 == Math.round(flat * 1000) / 1000.0) ? invert : flat);
                    }
                    pressedX = false;
                } else {
                    pressedX = true;
                }


                if (-gamepad2.right_stick_y > 0.1) {
                    robot.HangMotor.setPower(1);
                } else if (-gamepad2.right_stick_y < -0.1) {
                    robot.HangMotor.setPower(-1);
                } else {
                    robot.HangMotor.setPower(0);
                }

//            if(gamepad2.a){
//                if(pressedA){
//                    robot.LeftInTake.setPosition((Math.round(robot.LeftInTake.getPosition() * 1000) / 1000.0 == Math.round(leftClosed * 1000) / 1000.0) ? leftOpen : leftClosed);
//                    robot.RightInTake.setPosition((Math.round(robot.RightInTake.getPosition() * 1000) / 1000.0 == Math.round(rightClosed * 1000) / 1000.0) ? rightOpen : rightClosed);
//                }
//                pressedA = false;
//            }else{
//                pressedA = true;
//            }

//            if(gamepad2.a){
//                robot.openLeft();
//                robot.openRight();
//            } else {
//                robot.closeLeft();
//                robot.openRight();
//            }

                if (gamepad2.left_bumper) {
                    robot.openLeft();
                } else {
                    robot.closeLeft();
                }

                if (gamepad2.right_bumper) {
                    robot.openRight();
                } else {
                    robot.closeRight();
                }

//            if(gamepad2.left_bumper){
//                if(pressingLB){
//                    robot.LeftInTake.setPosition((Math.round(robot.LeftInTake.getPosition() * 1000) / 1000.0 == Math.round(leftClosed * 1000) / 1000.0) ? leftOpen : leftClosed);
//                }
//                pressingLB = false;
//            }else{
//                pressingLB = true;
//            }
//
//            if(gamepad2.right_bumper){
//                if(pressingRB){
//                    robot.RightInTake.setPosition((Math.round(robot.RightInTake.getPosition() * 1000) / 1000.0 == Math.round(rightClosed * 1000) / 1000.0) ? rightOpen : rightClosed);
//                }
//                pressingRB = false;
//            }else{
//                pressingRB = true;
//            }

                if (gamepad2.y) {
                    robot.AirplaneMotor.setPower(-1);
                    resetRuntime();
                    while (getRuntime() < .5) {
                    }
                    robot.AirplaneServo.setPosition(airplaneEngaged);
                    while (getRuntime() < 1) {
                    }
                    robot.AirplaneMotor.setPower(0);
                }
            }
        }
    }
