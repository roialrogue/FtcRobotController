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
        boolean pressingUpClaw = false;

        while (opModeIsActive()) {


            double forward;
            double strafing;
            double turning;
            double arm;

            forward = -gamepad1.left_stick_y;
            strafing = gamepad1.left_stick_x;
            turning = gamepad1.right_stick_x;
            arm = gamepad2.right_stick_y;

            double rfm = (forward - strafing - turning);
            double lfm = (forward + strafing + turning);
            double rbm = (forward + strafing - turning);
            double lbm = (forward - strafing + turning);

            double max = Math.max(Math.abs(rfm), Math.max(Math.abs(lfm), Math.max(Math.abs(rbm), Math.abs(lbm))));
            if (max < robot.maxSpeed) {
                robot.setPower(rfm, lfm, rbm, lbm);
            } else {
                double scaleFactor = robot.maxSpeed / max;
                robot.setPower((rfm) * scaleFactor, (lfm) * scaleFactor, (rbm) * scaleFactor, (lbm) * scaleFactor);
            }

            //To start !GAMEPAD1! press "A + Start" at the same time
            //To start !GAMEPAD2! press "B + Start" at the same time

            double clawOpen = 0.65;
            double clawClose = 0.55;
            if (gamepad2.right_trigger > 0.1) {
                if (!pressingOpenClaw) {
                    robot.AServo.setPosition(((int) (robot.AServo.getPosition() * 10) == (int) (clawClose * 10)) ? clawOpen : clawClose);
                }
                pressingOpenClaw = true;
            } else {
                pressingOpenClaw = false;
            }

            double armScale = 0.25;
            robot.AMotor1.setPower(gamepad2.right_stick_y * armScale);

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

                double launcherOpen = 0.18;
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
                }

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

