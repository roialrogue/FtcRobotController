package org.firstinspires.ftc.teamcode.Autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Pipelines.GPTCamera;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.Pipelines.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Pipelines.myGamePad;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Test Edit Auto")
public class ZP1BlueAuto extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime();
    Hardware robot = Hardware.getInstance();
    OpenCvCamera webCam;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    private GPTCamera detector;
    boolean isBlue;
    boolean editingConfig = true;
    boolean parkingInside = true;
    boolean cyclingNum2 = true;
    boolean cycling = true;
    int waitTime = 0;

    enum EditingMode { None, WaitTime, Cycling, CyclingNum2, ParkingInside};

    public void runOpMode() {
        robot.init(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        myGamePad myGamepad = new myGamePad( gamepad1) ;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        GPTCamera detector = new GPTCamera(false, telemetry);
        webCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webCam.openCameraDevice();
        FtcDashboard.getInstance().startCameraStream(webCam, 0);
        webCam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);

        webCam.setPipeline(detector);

        EditingMode editingMode = EditingMode.None;

        while (editingConfig) {
            if (myGamepad.isXPressed()) {
                switch( editingMode ) {
                    case None: editingMode = EditingMode.WaitTime; break;
                    case WaitTime: editingMode = EditingMode.Cycling; break;
                    case Cycling: editingMode = EditingMode.CyclingNum2; break;
                    case CyclingNum2: editingMode = EditingMode.ParkingInside; break;
                    case ParkingInside: editingMode = EditingMode.None; break;
                }
            }

            switch(editingMode) {
                case None:
                    break;
                case WaitTime:
                    if (myGamepad.isRightBumperPressed()) {
                        waitTime = waitTime + 500;
                    } else if (myGamepad.isLeftBumperPressed() && !(waitTime == 0)) {
                        waitTime = waitTime - 500;
                    }
                    break;

                case Cycling:
                    if (myGamepad.isRightBumperPressed()) {
                        cycling = true;
                    } else if (myGamepad.isLeftBumperPressed()) {
                        cycling = false;
                    }
                    break;

                case CyclingNum2:
                    if (myGamepad.isRightBumperPressed()) {
                        cyclingNum2 = true;
                    } else if (myGamepad.isLeftBumperPressed()) {
                        cyclingNum2 = false;
                    }
                    break;

                case ParkingInside:
                    if (myGamepad.isRightBumperPressed()) {
                        parkingInside = true;
                    } else if (myGamepad.isLeftBumperPressed()) {
                        parkingInside = false;
                    }
                    break;
            }
                telemetry.addData("Time the robot is waiting", waitTime);
                telemetry.addData("Cycling", cycling);
                telemetry.addData("We are cycling twice", cyclingNum2);
                telemetry.addData("Parking on the inside", parkingInside);
                telemetry.update();

                if (myGamepad.isleftstickbuttonPressed()) {
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
            //on the right side
            if (cycling) {
                if(cyclingNum2) {

                } else {

                }
            } else {

            }
        } else if(GPTCamera.middleSide) {
            //on the middle side
            if (cycling) {
                if(cyclingNum2) {

                } else {

                }
            } else {

            }
        } else if(GPTCamera.leftSide) {
            //on the left side
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

        PoseStorage.currentPose = drive.getPoseEstimate();
        }
    }
