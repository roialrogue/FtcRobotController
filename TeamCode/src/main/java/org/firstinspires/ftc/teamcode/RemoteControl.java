package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp (name = "TeleOp")
public class RemoteControl extends LinearOpMode {

    //Config Variables
    // RF = "CM0"
    // RB = "CM1"
    // LF = "CM2"
    // LB = "CM3"

    Hardware robot = Hardware.getInstance();

    public void runOpMode() {

        robot.init(hardwareMap);
        telemetry.addData("Status", "Please wo- ope, it worked ");
        telemetry.update();

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


        waitForStart();

        boolean pressingChangeLauncher = false;
        boolean pressingOpenLauncher = false;
        boolean pressingOpenClaw = false;

        while (opModeIsActive()) {

            double armUpDown = 0;
            double armExtend = 0;

            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;
            boolean clawIntake = gamepad2.right_bumper;
            boolean clawOutake = gamepad2.left_bumper;

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

            if (gamepad1.right_bumper) {
                lfm = gamepad1.x ? 1.0 : 0.0;
                lbm = gamepad1.a ? 1.0 : 0.0;
                rfm = gamepad1.y ? 1.0 : 0.0;
                rbm = gamepad1.b ? 1.0 : 0.0;
            }
            double speed;
            if(gamepad1.left_bumper) {
                speed = .6;
            } else {
                speed = 1;
            }

            robot.rightForwardWheel.setPower(rfm * speed);
            robot.rightRearWheel.setPower(rbm * speed);
            robot.leftForwardWheel.setPower(lfm * speed);
            robot.leftRearWheel.setPower(lbm * speed);

            if (clawIntake == true) {
                if (!pressingOpenClaw) {
                    robot.AMotorIntake.setPower(1);
                    pressingOpenClaw = true;
                }
            } else {
                pressingOpenClaw = false;
                robot.AMotorIntake.setPower(0);
            }
            if (clawOutake == true) {
                if (!pressingOpenClaw) {
                    robot.AMotorIntake.setPower(-1);
                    pressingOpenClaw = true;
                }
                pressingOpenClaw = true;
            } else {
                pressingOpenClaw = false;
                robot.AMotorIntake.setPower(0);

            }

            double armScale = 0.5;
            double power1 = armUpDown * armScale;
            double power2 = armExtend * armScale;

            if (gamepad2.left_stick_y > 0.1) {
                robot.AMotorUpDown.setPower(0.5);
            } else if (gamepad2.left_stick_y < -0.1) {
                robot.AMotorUpDown.setPower(-0.5);
            } else {
                robot.AMotorUpDown.setPower(0);
            }

            if (gamepad2.right_stick_y > 0.1) {
                robot.AMotorOutIn.setPower(0.5);
            } else if (gamepad2.right_stick_y < -0.1) {
                robot.AMotorOutIn.setPower(-0.5);
            } else {
                robot.AMotorOutIn.setPower(0);
            }

            robot.AMotorUpDown.setPower(armUpDown);
            robot.AMotorOutIn.setPower(armExtend);

//            double armUp = 0.25;
//            double armDown = -0.25;
//            if (arm > 0.1) {
//                if (!pressingUpClaw) {
//                    robot.AMotor1.setPower(armUp);
//                    pressingUpClaw = true;
//                }
//            } else if (arm < 0.1) {
//                if (!pressingOpenClaw) {
//                    robot.AMotor1.setPower(armDown);
//                    pressingUpClaw = false;
//                }
//            }

           /* double launcherOpen = 0.18;
            double launcherClose = 0.001;
            if (gamepad2.b) {
                if (!pressingOpenLauncher) {
                    robot.PServo1.setPosition(((int) (robot.PServo1.getPosition() * 10) == (int) (launcherClose * 10)) ? launcherOpen : launcherClose);
                }
                pressingOpenLauncher = true;
            } else {
                pressingOpenLauncher = false;
            }

            double launcherUp = 0.54;
            double launcherDown = 0.37;
            if (gamepad2.left_bumper) {
                if (!pressingChangeLauncher) {
                    robot.PServo2.setPosition(((int) (robot.PServo2.getPosition() * 10) == (int) (launcherDown * 10)) ? launcherUp : launcherDown);
                }
                pressingChangeLauncher = true;
            } else {
                pressingChangeLauncher = false;
            }*/

//            if ((gamepad2.left_bumper) && !pressingLB && !pressedLB) {
//                robot.PServo2.setPosition(0.54);
//                pressingLB = true;
//                pressedLB = true;
//            } else if ((gamepad2.left_bumper) && !pressingLB && pressedLB) {
//                robot.PServo2.setPosition(0.37);
//                pressingLB = true;
//                pressedLB = false;
//            } else if (!(gamepad2.left_bumper)) {
//                pressingLB = false;
//            }

                }
            }
    }





