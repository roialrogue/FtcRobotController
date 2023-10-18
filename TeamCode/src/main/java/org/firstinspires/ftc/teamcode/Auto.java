package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous (name = "AutoX")
public class Auto extends LinearOpMode {

    //Config Variables
    // RF = "CM0"
    // RB = "CM1"
    // LF = "CM2"
    // LB = "CM3"

    private ElapsedTime runtime = new ElapsedTime();
    Hardware robot = Hardware.getInstance();
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    OpenCvCamera webCam;
    private String position = "Level 1";
    private Pipeline detector;

    public void runOpMode() {

        robot.init(hardwareMap, true);
        telemetry.addData("Status", "(Metal Pipe Noise)");
        telemetry.update();

        int cameraMonitorViewID = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewID", "id", hardwareMap.appContext.getPackageName());
        detector = new Pipeline();
        webCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Camera"), cameraMonitorViewID);
        webCam.openCameraDevice();
        FtcDashboard.getInstance().startCameraStream(webCam, 0);
        webCam.startStreaming(640,480, OpenCvCameraRotation.UPSIDE_DOWN);
        webCam.setPipeline(detector);

        while (!isStarted() && !isStopRequested()) {

            position = detector.position;
            telemetry.addData("Position", position);
            telemetry.addData("totalA", detector.totalA);
            telemetry.update();

            dashboardTelemetry.addData("position", position);
            dashboardTelemetry.addData("totalA", detector.totalA);
            telemetry.update();

        }

        waitForStart();


        move(10, 0.6);

        if (position.equals("Location 1")) {
            turning(90);
            move(5,0.6);
        } else if (position.equals("Location 2")) {
            move(5, 0.6);
        } else if (position.equals("Location 3")){
            turning(-90);
            move(5,0.6);
        }



        /*
        runtime.reset();
        while (runtime.seconds() < 5) {
            robot.setPower(0.5,0.5,0.5,0.5);
        }
        robot.setPower(0,0,0,0);
         */


    }

    public void move(double distanceMoving, double speedMoving) {

        double wheelCircumference = 4 * Math.PI;
        double gearRatio = 560;
        double ticks = (distanceMoving * (gearRatio / wheelCircumference));

        robot.setPower(0, 0, 0, 0);

        robot.rightForwardWheel.setTargetPosition((int) Math.round(ticks));
        robot.leftForwardWheel.setTargetPosition((int) Math.round(ticks));
        robot.rightRearWheel.setTargetPosition((int) Math.round(ticks));
        robot.leftRearWheel.setTargetPosition((int) Math.round(ticks));

        robot.rightForwardWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftForwardWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightRearWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftRearWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.rightForwardWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftForwardWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightRearWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftRearWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.setPower(speedMoving, speedMoving, speedMoving, speedMoving);

        while (opModeIsActive() && robot.rightForwardWheel.getCurrentPosition() + 10 < ticks || robot.rightForwardWheel.getCurrentPosition() - 10 > ticks) {

        }

        robot.setPower(0, 0, 0, 0);

        robot.rightForwardWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftForwardWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightRearWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftRearWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

    public void turning(double degrees) {

        robot.rightForwardWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftForwardWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightRearWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftRearWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double currentAngle = robot.gyro.getAngularOrientation().firstAngle;
        double finalAngle = currentAngle + degrees;

        if (finalAngle > 180) {
            finalAngle -= 360;
        } else if (finalAngle < -180) {
            finalAngle += 360;
        }

        if (degrees >= 0) {

            double errorOfDegrees = degrees;
            while (Math.abs(errorOfDegrees) > 5) {
                robot.setPower(-0.0055 * errorOfDegrees, -0.0055 * errorOfDegrees, 0.0055 * errorOfDegrees, 0.0055 * errorOfDegrees);
                errorOfDegrees = finalAngle - robot.gyro.getAngularOrientation().firstAngle;
                if (errorOfDegrees > 180) {
                    errorOfDegrees -= 360;
                } else if (errorOfDegrees < -180) {
                    errorOfDegrees += 180;
                }
                errorOfDegrees = Math.abs(errorOfDegrees);

            }

        } else {
            double errorOfDegrees = degrees;
            while (Math.abs(errorOfDegrees) > 5) {
                robot.setPower(0.0055 * errorOfDegrees, 0.0055 * errorOfDegrees, -0.0055 * errorOfDegrees, -0.0055 * errorOfDegrees);
                errorOfDegrees = finalAngle - robot.gyro.getAngularOrientation().firstAngle;
                if (errorOfDegrees > 180) {
                    errorOfDegrees -= 360;
                } else if (errorOfDegrees < -180) {
                    errorOfDegrees += 180;
                }
                errorOfDegrees = Math.abs(errorOfDegrees);

            }

        }

        robot.setPower(0, 0, 0, 0);

    }


}

