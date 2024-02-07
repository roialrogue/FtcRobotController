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

        int editingWait = 0;
        int editingCycling = 0;
        int editingParking = 0;
        boolean parkingInside = false; // set default
        boolean parkingOutside = false;
        int waitTime = 0;
        boolean Cycling = false;

        if (gamepad1.a) {
            editingWait = editingWait + 1;
            if (editingWait % 2 == 0) {
                while (editingWait % 2 == 0) {
                    if (gamepad1.left_bumper) {
                        waitTime = waitTime + 500;
                    } else if (gamepad1.right_bumper && !(waitTime == 0)) {
                        waitTime = waitTime - 500;
                    }
                    telemetry.addData("Time the robot is waiting", waitTime);
                    telemetry.update();
                }
            }
        }

        if (gamepad1.b) {
            editingCycling = editingCycling + 1;
            if (editingCycling % 2 == 0) {
                while (editingCycling % 2 == 0) {
                    if (gamepad1.left_bumper) {
                        Cycling = true;
                    } else if (gamepad1.right_bumper && !(waitTime == 0)) {
                        Cycling = false;
                    }
                    telemetry.addData("Cycling", Cycling);
                    telemetry.update();
                }
            }
        }

        if (gamepad1.x) {
            editingParking = editingParking + 1;
            if (editingParking % 2 == 0) {
                while (editingParking % 2 == 0) {
                    if (gamepad1.left_bumper) {
                        parkingInside = true;
                        parkingOutside = false;
                    } else if (gamepad1.right_bumper && !(waitTime == 0)) {
                        parkingInside = false;
                        parkingOutside = true;
                    }
                    telemetry.addData("Parking on the inside", parkingInside);
                    telemetry.addData("parking on the outside", parkingOutside);
                    telemetry.update();
                }
            }
        }

        telemetry.addData("Time the robot is waiting", waitTime);
        telemetry.addData("Cycling", Cycling);
        telemetry.addData("Parking on the inside", parkingInside);
        telemetry.addData("parking on the outside", parkingOutside);

        waitForStart();
        sleep(waitTime);
        webCam.stopStreaming();

        if (Cycling == true) {

        } else if (Cycling == false) {

        }

        if (parkingOutside == true) {

        } else if (parkingInside == true) {

        }
    }
}