package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Pipelines.myGamePad;

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
        double invert = 0.0;

        robot.RightInTake.setPosition(rightOpen);
        robot.LeftInTake.setPosition(leftOpen);
        robot.ClawLeftRight.setPosition(flat);
        robot.ClawUpDown.setPosition(grounded);
        robot.AirplaneServo.setPosition(airplaneDisengaged);

        robot.BeltMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BeltMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

        waitForStart();

        boolean pressingRB = true;
        boolean pressingLB = true;
        boolean pressedX = true;
        boolean pressedA = true;


        while (opModeIsActive()) {

            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            //boolean beltSlowDrive = gamepad2.left_bumper;
            boolean airplane = gamepad2.y;
            boolean flip = gamepad2.x;

            double speed;
            if (gamepad1.left_bumper) {
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


            double Ticks = -robot.BeltMotor.getCurrentPosition();
            double minOrMax = 1;
            double beltSlowDriveSpeed = 1;
            double maxTicks = 537.6 * 5; //2850


            telemetry.addData("Belt Ticks", Ticks);
            telemetry.update();

            if (robot.BeltMotor.isBusy()) {
                if (Ticks < 100) {
                    minOrMax = 0.1;
                } else if (Ticks < 200) {
                    minOrMax = 0.33;
                } else if (Ticks < 400) {
                    minOrMax = 0.67;
                } else if (2400 < Ticks && Ticks < 2600) {
                    minOrMax = 0.33;
                } else if (2600 < Ticks && Ticks < maxTicks) {
                    minOrMax = 0.1;
                } else if (Ticks > maxTicks) {
                    minOrMax = 0.01;
                }
            }



            if (gamepad2.left_stick_y > 0.1) {
                robot.BeltMotor.setPower(1 * minOrMax);
            } else if (gamepad2.left_stick_y < -0.1) {
                robot.BeltMotor.setPower(-1 * minOrMax);
            } else {
                robot.BeltMotor.setPower(0);
            }

            if (Ticks <= 1200) {
                robot.ClawUpDown.setPosition(grounded);
                robot.ClawLeftRight.setPosition(flat);
            } else if (Ticks > 1600) {
                robot.ClawLeftRight.setPosition(invert);
                robot.ClawUpDown.setPosition(boardInvert);
            }

            if(gamepad2.x){
                if(pressedX){
                    robot.ClawUpDown.setPosition((Math.round(robot.ClawUpDown.getPosition() * 1000) / 1000.0 == Math.round(grounded * 1000) / 1000.0) ? boardInvert : grounded);
                    robot.ClawLeftRight.setPosition((Math.round(robot.ClawLeftRight.getPosition() * 1000) / 1000.0 == Math.round(flat * 1000) / 1000.0) ? invert : flat);
                }
                pressedX = false;
            }else{
                pressedX = true;
            }


            if (-gamepad2.right_stick_y > 0.1) {
                robot.HangMotor.setPower(1);
            } else if (-gamepad2.right_stick_y < -0.1) {
                robot.HangMotor.setPower(-1);
            } else {
                robot.HangMotor.setPower(0);
            }

            if(gamepad2.a){
                if(pressedA){
                    robot.LeftInTake.setPosition((Math.round(robot.LeftInTake.getPosition() * 1000) / 1000.0 == Math.round(leftClosed * 1000) / 1000.0) ? leftOpen : leftClosed);
                    robot.RightInTake.setPosition((Math.round(robot.RightInTake.getPosition() * 1000) / 1000.0 == Math.round(rightClosed * 1000) / 1000.0) ? rightOpen : rightClosed);
                }
                pressedA = false;
            }else{
                pressedA = true;
            }


            if(gamepad2.left_bumper){
                if(pressingLB){
                    robot.LeftInTake.setPosition((Math.round(robot.LeftInTake.getPosition() * 1000) / 1000.0 == Math.round(leftClosed * 1000) / 1000.0) ? leftOpen : leftClosed);
                }
                pressingLB = false;
            }else{
                pressingLB = true;
            }

            if(gamepad2.right_bumper){
                if(pressingRB){
                    robot.RightInTake.setPosition((Math.round(robot.RightInTake.getPosition() * 1000) / 1000.0 == Math.round(rightClosed * 1000) / 1000.0) ? rightOpen : rightClosed);
                }
                pressingRB = false;
            }else{
                pressingRB = true;
            }

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
