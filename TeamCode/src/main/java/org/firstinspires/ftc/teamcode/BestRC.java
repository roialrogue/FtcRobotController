package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Pipelines.myGamePad;

@TeleOp(name = "BestTeleOp")
public class BestRC extends LinearOpMode {

    Hardware robot = Hardware.getInstance();

    boolean rightIsClosed = false;
    boolean pressingRB = false;

    boolean leftIsClosed = false;
    boolean pressingLB = false;

    public void runOpMode() {
        myGamePad myGamepad = new myGamePad(gamepad1);
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
        double rightClosed = 0.48;

        double leftOpen = 0.95;
        double leftClosed = 0.65;

        double grounded = 0.505;
        double board = 0.7;
        double boardInvert = 0.12;

        double flat = 0.72;
        double invert = 0.1;

        robot.RightInTake.setPosition(rightOpen);
        robot.LeftInTake.setPosition(leftOpen);
        robot.ClawLeftRight.setPosition(flat);
        robot.ClawUpDown.setPosition(grounded);
        robot.AirplaneServo.setPosition(airplaneDisengaged);

        robot.BeltMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BeltMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);


        waitForStart();
        boolean slowDrive = gamepad1.left_bumper;

        while (opModeIsActive()) {

            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            double belts = -gamepad2.left_stick_y;
            //boolean beltSlowDrive = gamepad2.left_bumper;
            boolean leftIntake = myGamepad.isLeftBumperPressed();
            boolean rightIntake = gamepad2.right_bumper;
            boolean doubleIntake = myGamepad.isAPressed();
            boolean airplane = gamepad2.y;
            double hangArm = -gamepad2.right_stick_y;
            boolean flip = gamepad2.x;

            double speed;
            if (slowDrive) {
                speed = 0.33;
            } else {
                speed = .9;
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
            double minOrMax = 1;
            double beltSlowDriveSpeed = 1;

            telemetry.addData("Belt Ticks", Ticks);
            telemetry.update();

            if (robot.BeltMotor.isBusy()) {
                if (Ticks <= 100) {
                    minOrMax = 0.25;
                } else if (Ticks <= 200) {
                    minOrMax = 0.5;
                } else if (Ticks <= 400) {
                    minOrMax = 0.75;
                }
            }

            double maxTicks = 537.6 * 5.15; //5.15 rotations till max


            if (belts > 0.1 && Ticks <= maxTicks) {
                robot.BeltMotor.setPower(1 * minOrMax * beltSlowDriveSpeed);
            } else if (belts < -0.1 && Ticks >= 200) {
                robot.BeltMotor.setPower(-1 * minOrMax * beltSlowDriveSpeed);
            } else {
                robot.BeltMotor.setPower(0);
            }

//            if (Ticks <= 1200) {
//                robot.ClawUpDown.setPosition(grounded/2);
//                robot.ClawLeftRight.setPosition(flat/2);
//                robot.ClawUpDown.setPosition(grounded);
//                robot.ClawLeftRight.setPosition(flat);
//            } else if (Ticks > 1200) {
//                robot.ClawUpDown.setPosition(boardInvert/2);
//                robot.ClawLeftRight.setPosition(invert/2);
//                robot.ClawUpDown.setPosition(boardInvert);
//                robot.ClawLeftRight.setPosition(invert);
//            }


            if (hangArm > 0.1) {
                robot.HangMotor.setPower(1);
            } else if (hangArm < -0.1) {
                robot.HangMotor.setPower(-1);
            } else {
                robot.HangMotor.setPower(0);
            }


            if (gamepad2.left_bumper && !pressingLB) {
                if(!leftIsClosed) {
                    robot.LeftInTake.setPosition(leftClosed);
                    leftIsClosed = true;
                }
                 else {
                    robot.LeftInTake.setPosition(leftOpen);
                    leftIsClosed = false;
                }
                pressingLB = true;
            }

            else {
                pressingLB = false;
            }

            if(gamepad2.right_bumper && !pressingRB) {
                if (!rightIsClosed)
                    robot.RightInTake.setPosition(rightClosed);
                else
                    robot.RightInTake.setPosition(rightOpen);
                pressingRB = true;
                rightIsClosed = !rightIsClosed;
            } else{
                pressingRB = false;
            }


//            if (leftIntake && !leftPressed && !leftPressing) {
//               robot.LeftInTake.setPosition(leftClosed);
//               leftPressed = true;
//               leftPressing = true;
//            } else if (leftIntake && leftPressed && !leftPressing) {
//                robot.LeftInTake.setPosition(leftOpen);
//                leftPressed = false;
//                leftPressing = true;
//            } else if (!leftIntake && leftPressing) {
//                leftPressing = false;
//            }
//
//
//            if (rightIntake && !rightPressed && !rightPressing) {
//                robot.RightInTake.setPosition(rightClosed);
//                rightPressed = true;
//                rightPressing = true;
//            } else if (rightIntake && rightPressed && !rightPressing) {
//                robot.RightInTake.setPosition(rightOpen);
//                rightPressed = false;
//                rightPressing = true;
//            } else if (rightIntake && rightPressing) {
//                rightPressing = false;
//            }

            if (airplane) {
                robot.AirplaneMotor.setPower(-1);
                resetRuntime();
                while(getRuntime() < .5) { }
                robot.AirplaneServo.setPosition(airplaneEngaged);
                while(getRuntime() < 1) { }
                robot.AirplaneMotor.setPower(0);
            }
        }
    }
}
