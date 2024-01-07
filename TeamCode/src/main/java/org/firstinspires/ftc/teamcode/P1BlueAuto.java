package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "P.1 Blue Auto")
public class P1BlueAuto extends LinearOpMode{
    boolean isBlue;
    Hardware robot = Hardware.getInstance();
    ElapsedTime runtime = new ElapsedTime();
        public void runOpMode() {
            robot.init(hardwareMap);
            final boolean[] cameraWorked = {false};

            Hardware.getInstance().camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    GPTCamera cameraPipeline = new GPTCamera(true);
                    Hardware.getInstance().camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
                    Hardware.getInstance().camera.setPipeline(cameraPipeline);
                    telemetry.addData("Webcam has initialized correctly", "");
                    telemetry.update();
                    cameraWorked[0] = true;
                }

                @Override
                public void onError(int errorCode) {
                    telemetry.addData("Camera has broken", "");
                    telemetry.update();
                }
            });

            waitForStart();
            /*runtime.reset();
            while(runtime.seconds() < 5) {
            hw.leftForwardWheel.setPower(1);
            }
            hw.leftForwardWheel.setPower(0);*/

            if (cameraWorked[0]) {
                sleep(100); // Adjust this sleep time as needed
                //GPTCamera.Location location = cameraPipeline.getLocation();
                location = Hardware.getInstance().GPTcamera.getLocation();
                Hardware.getInstance().camera.stopStreaming(); //Watch this line
            }

            if (location == GPTCamera.Location.RIGHT) {
                telemetry.addData("Found on the right", "");

            } else if (location == GPTCamera.Location.MIDDLE) {
                telemetry.addData("Found on the middle", "");

            } else if (location == GPTCamera.Location.LEFT) {
                telemetry.addData("Found on the left", "");

            } else if (location == GPTCamera.Location.NOT_FOUND) {
                telemetry.addData("Did not find","");
                telemetry.update();
            }
        }
    }