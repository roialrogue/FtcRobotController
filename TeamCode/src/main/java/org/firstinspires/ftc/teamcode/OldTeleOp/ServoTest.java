package org.firstinspires.ftc.teamcode.OldTeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware;

@TeleOp (name = "ServoTest")
public class ServoTest extends LinearOpMode {

    Hardware robot = Hardware.getInstance();

    public void runOpMode() {

        robot.init(hardwareMap);
        telemetry.addData("Status", "stuff");
        telemetry.update();

        waitForStart();

        double position = 0;
        boolean pressingA = false;
        boolean pressingB = false;

        while (opModeIsActive()) {
            telemetry.addData("Position", position);
            telemetry.addData("ActualMotorPosition", robot.ClawRotationServo.getPosition());
            telemetry.update();

            robot.ClawRotationServo.setPosition(position);

            if (gamepad1.a && !pressingA) {
                position += 0.01;
                pressingA = true;
            } else if (!gamepad1.a) {
                pressingA = false;
            }
            if (gamepad1.b && !pressingB) {
                position -= 0.01;
                pressingB = true;
            } else if (!gamepad1.b) {
                pressingB = false;
            }




        }


    }


}
