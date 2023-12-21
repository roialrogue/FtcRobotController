package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp (name = "TeleOp")
public class RemoteControlWorking extends LinearOpMode {

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

        /*if (robot.rightForwardWheel != null) {
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
        }*/


        waitForStart();

        boolean pressingOpenClaw = false;

        while (opModeIsActive()) {


            double armUpDown;
            double armExtend;
            boolean clawIntake;
            boolean clawOutake;

            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            clawIntake = gamepad2.right_bumper;
            clawOutake = gamepad2.left_bumper;

            double rfm = axial - lateral - yaw;
            double rbm = axial + lateral - yaw;
            double lfm = axial + lateral + yaw;
            double lbm = axial - lateral + yaw;


            /*
            double rfm = (forward - strafing - turning); // Updated calculation for right front motor
            double rbm = (forward + strafing - turning); // Updated calculation for right back motor
            double lfm = (forward + strafing + turning); // Updated calculation for left front motor
            double lbm = (forward - strafing + turning); // Updated calculation for left back
            */

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

            robot.rightForwardWheel.setPower(rfm);
            robot.rightRearWheel.setPower(rbm);
            robot.leftForwardWheel.setPower(lfm);
            robot.leftRearWheel.setPower(lbm);


            //To start !GAMEPAD1! press "A + Start" at the same time
            //To start !GAMEPAD2! press "B + Start" at the same time

            // Intake Controls (IN AND OUT)
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

            // Arm Controls (UP AND DOWN)
            if (gamepad2.left_stick_y > 0.1) {
                robot.AMotorUpDown.setPower(0.5);
            } else if (gamepad2.left_stick_y < -0.1) {
                robot.AMotorUpDown.setPower(-0.5);
            } else {
                robot.AMotorUpDown.setPower(0);
            }

            // Arm Controls (OUT AND IN)
            if (gamepad2.right_stick_y > 0.1) {
                robot.AMotorOutIn.setPower(0.5);
            } else if (gamepad2.right_stick_y < -0.1) {
                robot.AMotorOutIn.setPower(-0.5);
            } else {
                robot.AMotorOutIn.setPower(0);
            }



        }
    }
}




