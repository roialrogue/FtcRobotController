/*
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "P.2 Red Auto")
public class P2RedAuto extends LinearOpMode{
    boolean isBlue;
    Hardware robot = Hardware.getInstance();
    ElapsedTime runtime = new ElapsedTime();


    public void runOpMode() {
        robot.init(hardwareMap);
        CameraInitialization cameraPipeline = new CameraInitialization(telemetry, false);
        boolean isBlue = true;
        Hardware hw = Hardware.getInstance();

        final boolean[] cameraWorked = new boolean[1];

        hw.camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                cameraWorked[0] = true;
                hw.camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
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
        hw.camera.stopStreaming();


        if (cameraWorked[0]) {
            //runs if camera works
            CameraInitialization.Location location = cameraPipeline.getLocation();
            if (CameraInitialization.stoneRight) {
                //thing is on right
                telemetry.addData("Found on the right", "");
                telemetry.update();


            } else if (CameraInitialization.stoneMiddle) {
                //thing is on middle
                telemetry.addData("Found on the middle", "");
                telemetry.update();


            } else if (CameraInitialization.stoneLeft){
                //thing is on left
                telemetry.addData("Found on the left","");
                telemetry.update();




            }
        }
    }
}*/
