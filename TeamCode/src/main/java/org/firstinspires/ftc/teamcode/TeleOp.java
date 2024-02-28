package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Autos.ZP1BlueAuto;
import org.firstinspires.ftc.teamcode.Autos.ZP1RedAuto;
import org.firstinspires.ftc.teamcode.Pipelines.myGamePad;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp")
public class TeleOp extends LinearOpMode {

    Hardware robot = Hardware.getInstance();
    enum Mode { modeDown, modeUp}

    public void runOpMode() {
        robot.init(hardwareMap);
        myGamePad myGamepad = new myGamePad(gamepad2);

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

        double grounded = 0.50;
        double board = 0.7;
        double boardInvert = 0.08;

        double flat = 0.73;
        double invert = 0.05;

        robot.ClawLeftRight.setPosition(flat);
        robot.ClawUpDown.setPosition(grounded);
        robot.AirplaneServo.setPosition(airplaneDisengaged);
        robot.BeltMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BeltMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        Mode mode = Mode.modeDown;

        waitForStart();

        boolean pressedX = true;

        while (opModeIsActive()) {

            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

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

            double ticks = robot.BeltMotor.getCurrentPosition();
            if (gamepad2.left_stick_y < -0.1 && ticks <= 2850) {
                //Up
                if (ticks <= 2500) {
                    robot.BeltMotor.setPower(1);
                } else if (ticks > 2750) {
                    robot.BeltMotor.setPower(.1);
                } else if (ticks > 2700) {
                    robot.BeltMotor.setPower(.3);
                } else if (ticks > 2650) {
                    robot.BeltMotor.setPower(.5);
                } else if (ticks > 2600) {
                    robot.BeltMotor.setPower(.7);
                } else if (ticks > 2550) {
                    robot.BeltMotor.setPower(.9);
                }
            } else if (gamepad2.left_stick_y > 0.1 && ticks >= 5) {
                //Down
                if (ticks >= 300) {
                    robot.BeltMotor.setPower(-1);
                } else if (ticks <= 55) {
                    robot.BeltMotor.setPower(-.1);
                } else if (ticks <= 115) {
                    robot.BeltMotor.setPower(-.2);
                } else if (ticks <= 155) {
                    robot.BeltMotor.setPower(-.4);
                } else if (ticks <= 205) {
                    robot.BeltMotor.setPower(-.7);
                } else if (ticks <= 255) {
                    robot.BeltMotor.setPower(-.9);
                }
            } else {
                robot.BeltMotor.setPower(0);
            }

            switch(mode) {
                case modeDown:
                    if (ticks <= 1200) {
                        robot.ClawUpDown.setPosition(grounded);
                        robot.ClawLeftRight.setPosition(flat);
                    } else if (ticks > 1800 && !gamepad2.x) {
                        robot.ClawLeftRight.setPosition(invert);
                        robot.ClawUpDown.setPosition(boardInvert);
                    }
                    break;
                case modeUp:
                    if (ticks <= 1200) {
                        robot.ClawUpDown.setPosition(grounded);
                        robot.ClawLeftRight.setPosition(flat);
                    } else if (ticks > 1800 && !gamepad2.x) {
                        robot.ClawLeftRight.setPosition(flat);
                        robot.ClawUpDown.setPosition(board);
                    }
                    break;
            }


            if (gamepad2.x) {
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

            if (myGamepad.isXPressed()) {
                switch(mode) {
                    case modeDown: mode = Mode.modeUp; break;
                    case modeUp: mode = Mode.modeDown; break;
                }
            }
        }
    }
}
