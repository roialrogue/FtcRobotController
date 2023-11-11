package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name = "Blue Auto")
public class BlueAutonomous extends LinearOpMode {
boolean isBlue;
    public void runOpMode() {
        CameraInitialization cameraPipeline = new CameraInitialization(telemetry, true);
        boolean isBlue = true;
        Hardware hw = Hardware.getInstance();
        hw.init(hardwareMap);

        final boolean[] cameraWorked = new boolean[1];

        hw.camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                cameraWorked[0] = true;
                hw.camera.startStreaming(1280,720, OpenCvCameraRotation.UPRIGHT);
                hw.camera.setPipeline(cameraPipeline);

                telemetry.addData("Webcam has initialized correctly", "");
                telemetry.update();
            }

            @Override
            public void onError(int errorCode) {
                cameraWorked[0] = false;
                telemetry.addData("Camera has broken", "");
                telemetry.update();
            }

        });

        waitForStart();

        if(cameraWorked[0]){
            //runs if camera works
            CameraInitialization.Location location = cameraPipeline.getLocation();
            if(location == CameraInitialization.Location.RIGHT){
                //thing is on right
            }else if(location == CameraInitialization.Location.MIDDLE){

            }else{

            }
        }



        while(opModeIsActive()){

        }
    }


}
