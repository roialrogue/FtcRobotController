package org.firstinspires.ftc.teamcode.OldTeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Hardware;

@TeleOp(name = "EncoderTest")
public class EncoderTest extends LinearOpMode {

    Hardware robot = Hardware.getInstance();


    public void runOpMode() {

        robot.init(hardwareMap);
        telemetry.addData("Status", "(Metal Pipe Noise)");
        telemetry.update();

        waitForStart();

        int position = 0;

        boolean pressingA = false;
        boolean pressingB = false;




        while (opModeIsActive()) {


            robot.AMotorUpDown.setTargetPosition(position);
            //robot.AMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.AMotorUpDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.AMotorUpDown.setPower(1);

            if (gamepad1.a && !pressingA) {
            position += 3;
            //robot.AMotor1.setTargetPosition(-position);
            pressingA = true;
            } else if (!gamepad1.a) {
            pressingA = false;
            }

            if (gamepad1.b && !pressingB) {
            position -= 3;
            //robot.AMotor1.setTargetPosition(-position);
            pressingB = true;
            } else if (!gamepad1.b) {
            pressingB = false;
            }

            telemetry.addData("Position", position);
            telemetry.addData("ActualMotorPosition", robot.AMotorUpDown.getCurrentPosition());
            telemetry.update();


        }




    }




}
