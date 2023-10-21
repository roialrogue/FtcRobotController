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

        int position = 0;

        robot.AMotor1.setTargetPosition(position);
        robot.AMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.AMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.AMotor1.setPower(0.2);

        while (opModeIsActive()) {


            double forward;
            double strafing;
            double turning;

            double arm;

            forward = -gamepad1.left_stick_y;
            strafing = gamepad1.left_stick_x;
            turning = gamepad1.right_stick_x;

            arm = -gamepad2.left_stick_y;

            double rfm = forward - strafing - turning;
            double lfm = forward + strafing + turning;
            double rbm = forward + strafing - turning;
            double lbm = forward - strafing + turning;

            double max = Math.max(Math.abs(rfm), Math.max(Math.abs(lfm), Math.max(Math.abs(rbm), Math.abs(lbm))));
            if (max < robot.maxSpeed) {
                robot.setPower(rfm, lfm, rbm, lbm);
            } else {
                double scaleFactor = robot.maxSpeed /max;
                robot.setPower((rfm) * scaleFactor, (lfm) * scaleFactor, (rbm) * scaleFactor, (lbm) * scaleFactor);
            }

            //To start !GAMEPAD1! press "A + Start" at the same time
            //To start !GAMEPAD2! press "B + Start" at the same time

            if (gamepad2.a) {
                //Intake turns on (blah blah blah), after pressing stops turns off (blah blah blah)
            }

            /*
            boolean pressingB = false;
            boolean pressedB = false;
            */

            if (arm > 0) {
                position = 5;
                robot.AMotor1.setTargetPosition(position);
            } else if (arm < 0) {
                position = 95;
                robot.AMotor1.setTargetPosition(position);
            }

            /*
            if (gamepad2.b && !pressingB && !pressedB) {
                //Intake turns on
                pressingB = true;
                pressedB = true;
            } else if (!gamepad2.b && !pressingB && pressedB) {
                //Intake turns off
                pressingB = true;
                pressedB = false;
            } else if (!gamepad2.b) {
                pressingB = false;
            }
            */


            boolean pressingRB = false;
            boolean pressedRB = false;
            boolean pressingLB = false;
            boolean pressedLB = false;

            if (gamepad2.right_bumper && !pressingRB && !pressedRB) {
                robot.AServoR.setPosition(-0.3);
                pressingRB = true;
                pressedRB = true;
            } else if ((gamepad2.right_bumper) && !pressingRB && pressedRB) {
                robot.AServoR.setPosition(-0.425);
                pressingRB = true;
                pressedRB = false;
            } else if (!(gamepad2.right_bumper)) {
                pressingRB = false;
            }

            if ((gamepad2.left_bumper) && !pressingLB && !pressedLB) {
                robot.AServoL.setPosition(0.35);
                pressingLB = true;
                pressedLB = true;
            } else if ((gamepad2.left_bumper) && !pressingLB && pressedLB) {
                robot.AServoL.setPosition(0.6);
                pressingLB = true;
                pressedLB = false;
            }

            if (!(gamepad2.left_bumper)) {
                pressingLB = false;
            }


        }



    }



}
