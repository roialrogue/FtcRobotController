package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Autos.ZP1BlueAuto;
import org.firstinspires.ftc.teamcode.Autos.ZP1RedAuto;
import org.firstinspires.ftc.teamcode.Pipelines.myGamePad;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp")
public class TeleOp extends LinearOpMode {

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

        double grounded = 0.52;
        double board = 0.7;
        double boardInvert = 0.08;

        double flat = 0.725;
        double invert = 0.05;

        robot.ClawLeftRight.setPosition(flat);
        robot.ClawUpDown.setPosition(grounded);
        robot.AirplaneServo.setPosition(airplaneDisengaged);
        robot.BeltMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BeltMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

        waitForStart();

        myGamePad myGamepad = new myGamePad(gamepad1);
        enum mode { modeDown, modeUp}
        mode mode = mode.modeDown;

        if (myGamepad.isXPressed()) {
            switch( mode ) {
                case modeDown: mode = mode.modeUp; break;
                case modeUp: mode = mode.modeDown; break;
            }
        }
        switch(mode) {
            case modeDown:

                break;
            case modeUp:

                break;

        boolean pressedX = true;

        while (opModeIsActive()) {

            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;
            boolean mode = gamepad2.b;

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
            } else if (gamepad2.left_stick_y > 0.1 && ticks >= 10) {
                //Down
                if (ticks >= 300) {
                    robot.BeltMotor.setPower(-1);
                } else if (ticks <= 60) {
                    robot.BeltMotor.setPower(-.1);
                } else if (ticks <= 110) {
                    robot.BeltMotor.setPower(-.2);
                } else if (ticks <= 160) {
                    robot.BeltMotor.setPower(-.4);
                } else if (ticks <= 210) {
                    robot.BeltMotor.setPower(-.7);
                } else if (ticks <= 260) {
                    robot.BeltMotor.setPower(-.9);
                }
            } else {
                robot.BeltMotor.setPower(0);
            }

            //Here
            if (ticks <= 1200) {
                robot.ClawUpDown.setPosition(grounded);
                robot.ClawLeftRight.setPosition(flat);
            } else if (ticks > 1800 && !gamepad2.x) {
                robot.ClawLeftRight.setPosition(invert);
                robot.ClawUpDown.setPosition(boardInvert);
            }

            if (ticks <= 1200) {
                robot.ClawUpDown.setPosition(grounded);
                robot.ClawLeftRight.setPosition(flat);
            } else if (ticks > 1800 && !gamepad2.x) {
                robot.ClawLeftRight.setPosition(flat);
                robot.ClawUpDown.setPosition(board);
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
