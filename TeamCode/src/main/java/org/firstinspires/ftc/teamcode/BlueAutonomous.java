package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Camera Test")
public class BlueAutonomous extends LinearOpMode {

    public void runOpMode() {
        waitForStart();
        CameraInitialization camera = new CameraInitialization(telemetry);
    }
}
