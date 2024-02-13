package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Test Edit Auto")
public class ZP1BlueAuto extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime();
    Hardware robot = Hardware.getInstance();
    OpenCvCamera webCam;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    myGamePad myGamepad = new myGamePad( gamepad1) ;
    private GPTCamera detector;
    boolean isBlue;
    boolean editingConfig = true;
    boolean parkingInside = true;
    boolean cyclingNum2 = true;
    boolean cycling = true;
    int waitTime = 0;

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

        while (editingConfig) {
            if (myGamepad.isXPressed()) {
                if (myGamepad.rightBumperPressed) {
                    waitTime = waitTime + 500;
                } else if (myGamepad.leftBumperPressed && !(waitTime == 0)) {
                    waitTime = waitTime - 500;
                }
            }
            if (myGamepad.isYPressed()) {
                if (myGamepad.rightBumperPressed) {
                    cycling = true;
                } else if (myGamepad.leftBumperPressed) {
                    cycling = false;
                }
            }
            if (myGamepad.isBPressed()) {
                if (myGamepad.rightBumperPressed) {
                    cyclingNum2 = true;
                } else if (myGamepad.leftBumperPressed) {
                    cyclingNum2 = false;
                }
            }
            if (myGamepad.isAPressed()) {
                if (myGamepad.rightBumperPressed) {
                    parkingInside = true;
                } else if (myGamepad.leftBumperPressed) {
                    parkingInside = false;
                }
            }

            telemetry.addData("Time the robot is waiting", waitTime);
            telemetry.addData("Cycling", cycling);
            telemetry.addData("We are cycling twice", cyclingNum2);
            telemetry.addData("Parking on the inside", parkingInside);
            telemetry.update();

            if (myGamepad.leftStickButtonPressed) {
                editingConfig = false;
            }
        }

        telemetry.addData("Ready","Editing auto is done");
        telemetry.update();

        //make sure to reset encoders
        //set servos to positions

        Pose2d BlueP1 = new Pose2d(15, 61, Math.toRadians(270));
        drive.setPoseEstimate(BlueP1);

        waitForStart();
        webCam.stopStreaming();
        sleep(waitTime);

        if (GPTCamera.rightSide) {

            if (cycling) {
                if(cyclingNum2) {

                } else {

                }
            } else {

            }
        } else if(GPTCamera.middleSide) {

            if (cycling) {
                if(cyclingNum2) {

                } else {

                }
            } else {

            }
        } else if(GPTCamera.leftSide) {


            if (cycling) {
                if(cyclingNum2) {

                } else {

                }
            } else {

            }
        } else if(GPTCamera.nonSide) {
            telemetry.addData("You need to wait for the Camera to Initialize", "");
        }

        if (parkingInside) {

        } else {

            }
        }
    }