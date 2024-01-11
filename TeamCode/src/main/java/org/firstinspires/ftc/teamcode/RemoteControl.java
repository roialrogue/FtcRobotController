package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp (name = "TeleOp")
public class RemoteControl extends LinearOpMode {
    // 3 tickes = Drop
    // 2 tickes = Bottom roller
    // 1 ticke = intake roller
    // 0 tickes = angle

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


        double ServoUp = 0.521;
        double ServoDown = 0.243;
        robot.HangServo.setPosition(ServoDown);
        waitForStart();

        boolean pressingChangeLauncher = false;
        boolean pressingOpenLauncher = false;
        boolean pressingOpenClaw = false;

        while (opModeIsActive()) {

            robot.AMotorOutIn.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.AMotorUpDown.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;
            boolean slowDrive = gamepad1.left_bumper;

            double beltOutIn = gamepad2.right_stick_y;
            double armUpDown = gamepad2.left_stick_y;
            boolean hangArm = gamepad2.y;
            boolean armSlowDrive = gamepad2.left_bumper;
//          boolean beltSlowDrive = gamepad2.right_bumper;


            boolean clawDropRight = gamepad2.b;
            boolean clawDropLeft = gamepad2.a;
            double clawAngleRotationUp = gamepad2.left_trigger;
            double clawAngleRotationDown = gamepad2.right_trigger;


            double clawBootWheelIntake = gamepad1.left_trigger;
            double clawBarIntake = gamepad1.right_trigger;

            //Claw wrist joint stabilization
            double ticksPerRevolution = 537.6 * 20;
            double currentArmAngle = Math.round(360 * (robot.AMotorUpDown.getCurrentPosition()/ticksPerRevolution));

            double position = robot.ClawRotationServo.getPosition();
            if (currentArmAngle >= 20) {
                robot.ClawRotationServo.setPosition(position + 0.01 * (currentArmAngle - 20));
            }

            // Claw intake system
            if(clawBootWheelIntake > .5) {
                robot.InTakeServo1.setPosition(1);
                robot.InTakeServo2.setPosition(1);
            } else {
                robot.InTakeServo1.setPosition(0.5);
                robot.InTakeServo2.setPosition(0.5);
            }


            //pixel dropping
            if(clawDropLeft) {
                robot.ClawDropServo.setPosition(.930);
            } else if (clawDropRight) {
                robot.ClawDropServo.setPosition(0.040);
            } else
                robot.ClawDropServo.setPosition(0.500);

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

            if (slowDrive) {
                lfm = gamepad1.x ? 1.0 : 0.0;
                lbm = gamepad1.a ? 1.0 : 0.0;
                rfm = gamepad1.y ? 1.0 : 0.0;
                rbm = gamepad1.b ? 1.0 : 0.0;
            }
            double speed;
            if (gamepad1.left_bumper) {
                speed = 0.33;
            } else {
                speed = 1;
            }

            robot.rightForwardWheel.setPower(rfm * speed);
            robot.rightRearWheel.setPower(rbm * speed);
            robot.leftForwardWheel.setPower(lfm * speed);
            robot.leftRearWheel.setPower(lbm * speed);

            //Up and down arm
            double fastDrive = 1;
            if (armSlowDrive){
                fastDrive = 0.6;
            }

            if (armUpDown > 0.1) {
                    robot.AMotorUpDown.setPower(-1 * fastDrive);
            } else if (armUpDown < -0.1) {
                    robot.AMotorUpDown.setPower(1 * fastDrive);
            } else {
                robot.AMotorUpDown.setPower(0);
            }

            //In and out arm
            robot.AMotorOutIn.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            int currentArmPosition = robot.AMotorOutIn.getCurrentPosition() * -1;
            final int maxBeltOut = 3000;
            final int minBeltIn = 200;
            final int slowdownThreshold = 200;
            int remainingDistance = maxBeltOut - currentArmPosition;
            double powerScale = 0.5;

            if (remainingDistance <= slowdownThreshold && remainingDistance > 0) {
                powerScale = (double) remainingDistance / slowdownThreshold;
            }

            if (beltOutIn > 0.1 && currentArmPosition > minBeltIn) {
                //Claw motion in
                robot.AMotorOutIn.setPower(1 * powerScale);
            } else if (beltOutIn < -0.1 && currentArmPosition < maxBeltOut) {
                //Claw motion out
                robot.AMotorOutIn.setPower(1 * powerScale);
            } else {
                robot.AMotorOutIn.setPower(0);
            }
            telemetry.addData("How far is the arm out", currentArmPosition);
            telemetry.addData("Real Value", robot.AMotorOutIn.getCurrentPosition());
            telemetry.update();

            //Servo for hanging
            ServoDown = 0.243;
            if (hangArm == true) {
                robot.HangServo.setPosition(ServoUp);
            } else {
                robot.HangServo.setPosition(ServoDown);
            }


        }


    }


}