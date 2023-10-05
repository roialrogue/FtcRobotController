package org.firstinspires.ftc.teamcode.SummerCodingClass;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp (name = "TeleOp")
public class RemoteControl extends LinearOpMode {

    Hardware robot = Hardware.getInstance();

    public void runOpMode() {

        robot.init(hardwareMap);
        telemetry.addData("Status", "(Metal Pipe Noise)");
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

        while (opModeIsActive()) {

            double forward;
            double strafing;
            double turning;

            forward = gamepad1.right_stick_x;
            strafing = gamepad1.left_stick_y;
            turning = -gamepad1.left_stick_x;

            double max = Math.max(Math.abs(forward - strafing - turning), Math.max(Math.abs(forward + strafing - turning), Math.max(Math.abs(forward + strafing + turning), Math.abs(forward - strafing + turning))));
            if (max < robot.maxSpeed) {
                robot.setPower(forward - strafing - turning, forward + strafing - turning, forward + strafing + turning, forward - strafing + turning);
            } else {
                double scaleFactor = max / robot.maxSpeed;
                robot.setPower((forward - strafing - turning) * scaleFactor, (forward + strafing - turning) * scaleFactor, (forward + strafing + turning) * scaleFactor, (forward - strafing + turning) * scaleFactor);
            }

            //To start !GAMEPAD1! press "A + Start" at the same time
            //To start !GAMEPAD2! press "B + Start" at the same time

            if (gamepad2.a) {
                //Intake turns on (blah blah blah), after pressing stops turns off (blah blah blah)
            }

            boolean pressingB = false;
            boolean pressedB = false;

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

            boolean pressingRT = false;
            boolean pressedRT = false;

            if (gamepad2.right_trigger > 0.1 && !pressingRT && !pressedRT) {
                //Intake turns on
                pressingRT = true;
                pressedRT = true;
            } else if (gamepad2.right_trigger > 0.1 && !pressingRT && pressedRT) {
                //Intake turns off
                pressingRT = true;
                pressedRT = false;
            } else if (gamepad2.right_trigger < 0.1) {
                pressingRT = false;
            }


        }



    }



}
