package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "EncoderTest")
public class EncoderTest extends LinearOpMode {

    Hardware robot = Hardware.getInstance();


    public void runOpMode() {

        robot.init(hardwareMap, false);
        telemetry.addData("Status", "(Metal Pipe Noise)");
        telemetry.update();

        int position = 0;
        boolean pressingA = false;
        boolean pressingB = false;

        waitForStart();

        while (opModeIsActive()) {

        position = robot.demoMotor.getCurrentPosition();
        robot.demoMotor.setPower(1);
        robot.demoMotor.setTargetPosition(position);
        robot.demoMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            if (gamepad1.a && !pressingA) {
            position += 3;
            pressingA = true;
            } else if (!gamepad1.a) {
            pressingA = false;
            }
            if (gamepad1.b && !pressingB) {
            position -= 3;
            pressingB = true;
            } else if (!gamepad1.b) {
            pressingB = false;
            }

            telemetry.addData("Position", position);
            telemetry.addData("ActualMotorPosition", robot.demoMotor.getCurrentPosition());
            telemetry.update();


        }




    }




}
