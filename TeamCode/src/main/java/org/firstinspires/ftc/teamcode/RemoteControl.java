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

        robot.AMotorUpDown.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.AMotorOutIn.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        double ServoUp = 0.521;
        double ServoDown = 0.243;
        robot.HangServo.setPosition(ServoDown);
        robot.AMotorOutIn.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.AMotorUpDown.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {

            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x; //check turning
            boolean slowDrive = gamepad1.left_bumper;
            double clawWheelIntake = gamepad1.left_trigger;

            double beltOutIn = gamepad2.right_stick_y;
            boolean beltSlowDrive = gamepad2.right_bumper;
            double armUpDown = gamepad2.left_stick_y;
            boolean armSlowDrive = gamepad2.left_bumper;
            boolean hangArm = gamepad2.y;
            boolean clawDropRight = gamepad2.b; //test
            boolean clawDropLeft = gamepad2.a; //test
            boolean clawDropBoth = gamepad2.x;

            //Claw wrist joint stabilization
            double ticksPerRevolution = 537.6 * 20;
            double currentArmAngle = Math.round(360 * (robot.AMotorUpDown.getCurrentPosition() / ticksPerRevolution));

            telemetry.addData("Ticks", ticksPerRevolution);
            telemetry.addData("Arm Angle", currentArmAngle);
            telemetry.addData("Current Servo Pos", robot.ClawRotationServo.getPosition());
            telemetry.addData("Current Servo Angle", (robot.ClawRotationServo.getPosition() * 300));

            double targetAngle = 25.0; // Adjust this value based on your desired angle
            double basePosition = 0.61; // Adjust this value based on your servo's base position
            double baseAdjustment = 0.060; // Automatically adjusts from the base angle to allow linear angle function to start in correct position
            double scaleFactor = 0.0022; // Adjust this value based on how much you want the servo to move per degree (Serovs function on a 0 - 1 range)

            if (currentArmAngle >= targetAngle) {
                robot.ClawRotationServo.setPosition(basePosition - baseAdjustment - (currentArmAngle * scaleFactor));
            } else {
                robot.ClawRotationServo.setPosition(basePosition);
            }

/*            double desiredServoPosition = basePosition + scaleFactor * (currentArmAngle);

            desiredServoPosition = Math.max(0.0, Math.min(1.0, desiredServoPosition));

            boolean above30 = false;
            if(above30 == true)  {
                robot.ClawRotationServo.setPosition(desiredServoPosition);
            }
            if (currentArmAngle >= 30) {
                above30 = true;
            } else if (currentArmAngle < 30) {
                above30 = false;
                robot.ClawRotationServo.setPosition(basePosition);
            }*/


            //Drive code
            //Slow Drive
            double speed;
            if (slowDrive) {
                speed = .6;
            } else {
                speed = 1;
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

            //In and out arm
            //Slow Drive
            double SlowDriveArm = 1;
            if (beltSlowDrive){
                SlowDriveArm = 0.6;
            }

            if (beltOutIn > 0.1) {
                //Claw motion out
                robot.AMotorOutIn.setPower(1 * SlowDriveArm);
            } else if (beltOutIn < -0.1) {
                //Claw motion in
                robot.AMotorOutIn.setPower(-1 * SlowDriveArm);
            } else {
                robot.AMotorOutIn.setPower(0);
            }

            //Up and down arm
            //slow Drive
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

            // Claw intake system
            if(clawWheelIntake > .5) {
                robot.InTakeServo1.setPosition(1);
                robot.InTakeServo2.setPosition(1);
            } else {
                robot.InTakeServo1.setPosition(0.5);
                robot.InTakeServo2.setPosition(0.5);
            }

            //pixel dropping
            if(clawDropLeft) {
                robot.ClawDropServo.setPosition(0.115);
            } else if (clawDropRight) {
                robot.ClawDropServo.setPosition(0.785);
            } else if (clawDropBoth) {
                robot.ClawDropServo.setPosition(1);
            } else {
                robot.ClawDropServo.setPosition(0.500);
            }

            //Servo for hanging
            ServoDown = 0.243;
            if (hangArm == true) {
                robot.HangServo.setPosition(ServoUp);
            }
        }
    }
}