package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.security.cert.CertificateNotYetValidException;

@Autonomous(name = "Red P.1 Auto")
public class ZP1BlueAuto extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime();
    Hardware robot = Hardware.getInstance();
    OpenCvCamera webCam;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    boolean isBlue;
    private GPTCamera detector;

    public void runOpMode() {
        robot.init(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        GPTCamera detector = new GPTCamera(false, telemetry);
        webCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webCam.openCameraDevice();
        FtcDashboard.getInstance().startCameraStream(webCam, 0);
        webCam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
        webCam.setPipeline(detector);

        boolean editingConfig = true;
        boolean parkingInside = false; // set default
        boolean parkingOutside = false;
        int waitTime = 0;
        boolean cycling = false;

        while (editingConfig) {
            if (gamepad1.x) {
                if (gamepad1.dpad_left) {
                    waitTime = waitTime + 500;
                } else if (gamepad1.dpad_right && !(waitTime == 0)) {
                    waitTime = waitTime - 500;
                }
            }
            if (gamepad1.a) {
                if (gamepad1.dpad_left) {
                    parkingInside = true;
                    parkingOutside = false;
                } else if (gamepad1.dpad_right) {
                    parkingInside = false;
                    parkingOutside = true;
                }
            }
            if (gamepad1.b) {
                if (gamepad1.dpad_left) {
                    cycling = true;
                } else if (gamepad1.dpad_right) {
                    cycling = false;
                }
            }

            telemetry.addData("Time the robot is waiting", waitTime);
            telemetry.addData("Cycling", cycling);
            telemetry.addData("Parking on the inside", parkingInside);
            telemetry.addData("parking on the outside", parkingOutside);
            telemetry.update();

            if (gamepad1.left_stick_button) {
                editingConfig = false;
            }
        }


            waitForStart();
            sleep(waitTime);
            webCam.stopStreaming();

            if (cycling == true) {

            }

            if (parkingOutside == true) {

            } else if (parkingInside == true) {

            }
        }
    }