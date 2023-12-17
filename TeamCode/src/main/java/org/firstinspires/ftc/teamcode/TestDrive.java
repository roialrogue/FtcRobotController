package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "TestDrive")
public class TestDrive extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    Hardware robot = Hardware.getInstance();

    public void runOpMode() {
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initailized");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {

            double forward;
            double sideways;
            double turning;
            forward = gamepad1.left_stick_y;
            sideways = -gamepad1.left_stick_x;
            turning = -gamepad1.right_stick_x;

            double max = Math.max(Math.abs(forward - sideways - turning), Math.max(Math.abs(forward + sideways - turning), Math.max(Math.abs(forward + sideways + turning), Math.abs(forward + turning - sideways))));
            if (max < robot.maxSpeed) {
                robot.setPower(forward - sideways - turning, forward + sideways - turning, forward + sideways + turning, forward + turning - sideways);
            } else {
                double scaleFactor = max / robot.maxSpeed;
                robot.setPower((forward - sideways - turning) * scaleFactor, (forward + sideways - turning) * scaleFactor, (forward + sideways + turning) * scaleFactor, (forward + turning - sideways) * scaleFactor);
            }
        }
    }
}
