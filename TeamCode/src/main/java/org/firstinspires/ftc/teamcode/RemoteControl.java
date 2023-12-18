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


            double forward;
            double strafing;
            double turning;
            double armUpDown;
            double armExtend;
            boolean clawIntake;
            boolean clawOutake;

            forward = gamepad1.left_stick_y;
            strafing = gamepad1.right_stick_x;
            turning = gamepad1.left_stick_x;
            clawIntake = gamepad2.right_bumper;
            clawOutake = gamepad2.left_bumper;


            double rfm = (forward - strafing - turning); // Updated calculation for right front motor
            double rbm = (forward + strafing - turning); // Updated calculation for right back motor
            double lfm = (forward + strafing + turning); // Updated calculation for left front motor
            double lbm = (forward - strafing + turning); // Updated calculation for left back



            /*double max;
            max = Math.max(Math.abs(lfm) , Math.abs(rfm));
            max = Math.max(max, Math.abs(lbm));
            max = Math.max(max, Math.abs(rbm));*/

            /*if (max > 1.0) {
                lfm /= max;
                rfm /= max;
                lbm /= max;
                rbm /= max;
            }*/

            /*lfm = gamepad1.x ? 1.0 : 0.0;
            lbm = gamepad1.a ? 1.0 : 0.0;
            rfm = gamepad1.y ? 1.0 : 0.0;
            rbm = gamepad1.b ? 1.0 : 0.0;*/



            /*double rfm = (forward - strafing - turning);
            double rbm = (forward + strafing - turning);
            double lfm = (forward + strafing + turning);
            double lbm = (forward - strafing + turning);*/


            /*double max = Math.max(Math.abs(forward) + Math.abs(turning) + Math.abs(strafing), robot.maxSpeed);
            double rfmPower = rfm / max;
            double rbmPower = rfm / max;
            double lfmPower = rfm / max;
            double lbmPower = rfm / max;

            robot.setPower(rfmPower, rbmPower, lfmPower, lbmPower);*/


            /*double max = Math.max(Math.abs(rfm), Math.max(Math.abs(lfm), Math.max(Math.abs(rbm), Math.abs(lbm))));
            if (max < robot.maxSpeed) {
                robot.setPower(rfm, lfm, rbm, lbm);
            } else {
                double scaleFactor = robot.maxSpeed / max;
                robot.setPower((rfm) * scaleFactor, (lfm) * scaleFactor, (rbm) * scaleFactor, (lbm) * scaleFactor);
            }*/
            double max;
            double scaleFactor;
            max = Math.max(Math.abs(forward - strafing - turning), Math.max(Math.abs(forward + strafing - turning), Math.max(Math.abs(forward + strafing + turning), Math.abs(forward + turning - strafing))));
            if (max > robot.maxSpeed) {
                scaleFactor = robot.maxSpeed / max;
            } else {
                scaleFactor = robot.maxSpeed;
            }
            scaleFactor *= Math.max(Math.abs(1 - gamepad1.right_trigger), 0.2);
            robot.setPower((forward - strafing - turning) * scaleFactor, (forward + strafing - turning) * scaleFactor, (forward + strafing + turning) * scaleFactor, (forward + turning - strafing) * scaleFactor);
        }


            //To start !GAMEPAD1! press "A + Start" at the same time
            //To start !GAMEPAD2! press "B + Start" at the same time

            /*if (clawIntake == true) {
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

            }*/


            //double armScale = 0.5;
            //double power1 = armUpDown * armScale;
            //double power2 = armExtend * armScale;

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
            //robot.AMotorUpDown.setPower(armUpDown);
            //robot.AMotorOutIn.setPower(armExtend);

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

            telemetry.addData("Right Target Position", robot.rightForwardWheel.getTargetPosition());
            telemetry.addData("Right Real Position", robot.rightForwardWheel.getCurrentPosition());
            telemetry.addData("Right Back Target Position", robot.rightRearWheel.getTargetPosition());
            telemetry.addData("Right Back Real Position", robot.rightRearWheel.getCurrentPosition());
            telemetry.addData("Left Target Position", robot.leftForwardWheel.getTargetPosition());
            telemetry.addData("Left Real Position", robot.leftForwardWheel.getCurrentPosition());
            telemetry.addData("Left Back Target Position", robot.leftRearWheel.getTargetPosition());
            telemetry.addData("Left Back Real Position", robot.leftRearWheel.getCurrentPosition());
            telemetry.update();
        }


    }
