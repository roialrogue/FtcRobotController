package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name = "Red Auto")
public class RedAutonomous extends LinearOpMode {

    public void runOpMode() {
        CameraInitialization cameraPipeline = new CameraInitialization(telemetry, false);
        boolean isBlue = false;

        Hardware hw = Hardware.getInstance();
        hw.init(hardwareMap);

        hw.camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                hw.camera.startStreaming(1280,720, OpenCvCameraRotation.UPRIGHT);
                hw.camera.setPipeline(cameraPipeline);

                telemetry.addData("Webcam has initialized correctly", "");
                telemetry.update();
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera has broken", "");
                telemetry.update();
            }

        });


        waitForStart();
        while(opModeIsActive()){

        }
    }


}
