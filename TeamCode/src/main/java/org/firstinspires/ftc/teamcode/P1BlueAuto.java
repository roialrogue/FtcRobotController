package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Camera Test Blue")
public class P1BlueAuto extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime();
    Hardware robot = Hardware.getInstance();
    OpenCvCamera webCam;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
    boolean isBlue;

    private GPTCamera detector;

    public void runOpMode() {
        Hardware.getInstance().camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            private void initCamera() {
                int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
                GPTCamera detector = new GPTCamera(true);
                webCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
                webCam.openCameraDevice();
                FtcDashboard.getInstance().startCameraStream(webCam, 0);
                webCam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
                webCam.setPipeline(detector);


                waitForStart();

                if (GPTCamera.leftSide == true) {
                    telemetry.addData("Found in Auto on the", "right");
                } else if (GPTCamera.middleSide == true) {
                    telemetry.addData("Found in Auto on the", "middle");
                } else if (GPTCamera.rightSide == true) {
                    telemetry.addData("Found in Auto on the", "left");
                }
                telemetry.update();

            }
        }
    }
}



