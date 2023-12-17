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

        if (robot.rightForwardWheel != null) {
            robot.rightForwardWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if (robot.rightRearWheel != null) {
            robot.rightRearWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if (robot.leftForwardWheel != null) {
            robot.leftForwardWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if (robot.leftRearWheel != null) {
            robot.leftRearWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initailized");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {

        double forward;
        double sideways;
        double turning;
        double max;
        double scaleFactor = 0;

        forward =  gamepad1.left_stick_y;
        sideways = gamepad1.left_stick_x;
        turning =  gamepad1.right_stick_x;
        max = Math.max(Math.abs(forward - sideways - turning), Math.max(Math.abs(forward + sideways - turning), Math.max(Math.abs(forward + sideways + turning), Math.abs(forward + turning - sideways))));
        if (max > robot.maxSpeed) {
            scaleFactor = robot.maxSpeed / max;
        } else {
            scaleFactor = robot.maxSpeed;
        }
        scaleFactor *= Math.max(Math.abs(1 - gamepad1.right_trigger), 0.2);
        robot.setPower((forward - sideways - turning) * scaleFactor, (forward + sideways - turning) * scaleFactor, (forward + sideways + turning) * scaleFactor, (forward + turning - sideways) * scaleFactor);
        }

    }
}
