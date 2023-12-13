/*
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous (name = "Meet 1 Auto")
public class PAuto extends LinearOpMode {
    Hardware robot = Hardware.getInstance();
    OpenCvCamera webCam;
    private PipelineDemo detector;
    private String position = "Left";
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    public void runOpMode() {
        robot.init(hardwareMap);
        telemetry.addData("Status","Hello, drivers!");
        telemetry.update();

        int cameraMonitorViewID = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewID", "id", hardwareMap.appContext.getPackageName());
        detector = new PipelineDemo();
        webCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class,"wc1"), cameraMonitorViewID);
        webCam.openCameraDevice();
        FtcDashboard.getInstance().startCameraStream(webCam, 0);
        webCam.startStreaming(1280,720, OpenCvCameraRotation.UPRIGHT);
        webCam.setPipeline(detector);
        while (!isStarted() && !isStopRequested()) {
            position = detector.position;
            telemetry.addData("position",position);
            telemetry.addData("leftBlue", position);
        }

        if ((position.equals("Left"))) {

        } if ((position.equals("Right"))) {

        }


        waitForStart();
    }
}
*/
