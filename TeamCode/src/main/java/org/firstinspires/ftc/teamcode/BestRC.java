package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

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

        robot.RightInTake.setPosition(0.5);
        robot.LeftInTake.setPosition(0.9);

        waitForStart();


        while (opModeIsActive()) {

            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;
            boolean slowDrive = gamepad1.left_bumper;

            double belts = gamepad2.left_stick_y;
            boolean beltSlowDrive = gamepad2.left_bumper;
            double leftIntake = gamepad2.left_trigger;
            double rightIntake = gamepad2.right_trigger;
            boolean doubleIntake = gamepad2.x;
            boolean airplane = gamepad2.y;
            double hangArm = -gamepad2.right_stick_y;
            boolean flip = gamepad2.a;

            double airplaneDisengaged = 0.3;
            double airplaneEngaged = 0.12;


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


            int Ticks = robot.BeltMotor.getCurrentPosition();
            double minOrMax = 1;
            double beltSlowDriveSpeed = 1;

            if (beltSlowDrive) {
                beltSlowDriveSpeed = 0.6;
            }

            telemetry.addData("Belt Ticks", Ticks);
            telemetry.update();


            if (belts > 0.1 && Ticks <= 3000) {
                robot.BeltMotor.setPower(1 * minOrMax * beltSlowDriveSpeed);
            } else if (belts < -0.1 && Ticks >= 200) {
                robot.BeltMotor.setPower(-1 * minOrMax * beltSlowDriveSpeed);
            } else {
                robot.BeltMotor.setPower(0);
            }

            if (robot.BeltMotor.isBusy()) {
                if (Ticks <= 500) {
                    minOrMax = 0.66;
                } else if (Ticks <= 250){
                    minOrMax = 0.33;
                } else if (Ticks <= 100) {
                    minOrMax = 0.1;
                }
            }


            if (hangArm > 0.1) {
                robot.HangMotor.setPower(1);
            } else if (hangArm < -0.1) {
                robot.HangMotor.setPower(-1);
            } else {
                robot.HangMotor.setPower(0);
            }


            boolean leftPressed = false;
            boolean rightPressed = false;
            boolean leftPressing = false;
            boolean rightPressing = false;

            /*
            if (leftIntake && !leftPressed && !leftPressing) {
               robot.LeftInTake.setPosition(0.94);
               leftPressed = true;
               leftPressing = true;
            } else if (leftIntake && leftPressed && !leftPressing) {
                robot.LeftInTake.setPosition(0.7);
                leftPressed = false;
                leftPressing = true;
            } else if (!leftIntake && leftPressing) {
                leftPressing = false;
            }
            */

            if (rightIntake > 0.1 && !rightPressed && !rightPressing) {
                robot.RightInTake.setPosition(0.5);
                rightPressed = true;
                rightPressing = true;
            } else if (rightIntake > 0.1 && rightPressed && !rightPressing) {
                robot.RightInTake.setPosition(0.775);
                rightPressed = false;
                rightPressing = true;
            } else if (rightIntake < 0.1 && rightPressing) {
                rightPressing = false;
            }


            if (airplane) {
                robot.AirplaneMotor.setPower(1);
                resetRuntime();
                    while (getRuntime() < 1) {
                     robot.AirplaneServo.setPosition(airplaneDisengaged);
                    }
                resetRuntime();
                robot.AirplaneServo.setPosition(airplaneEngaged);
                while (getRuntime() < 1) {
                    robot.AirplaneServo.setPosition(airplaneEngaged);
                }
                robot.AirplaneMotor.setPower(0);
            } else {
                robot.AirplaneServo.setPosition(airplaneDisengaged);
                robot.AirplaneMotor.setPower(0);
            }
        }
    }
}
