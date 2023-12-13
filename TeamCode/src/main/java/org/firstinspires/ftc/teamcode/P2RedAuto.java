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
        robot.rightForwardWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightRearWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftForwardWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftRearWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        if (cameraWorked[0]) {
            //runs if camera works
            CameraInitialization.Location location = cameraPipeline.getLocation();
            if (CameraInitialization.stoneRight) {
                //thing is on right
                telemetry.addData("Found on the right", "");
                telemetry.update();
                move(30,.4);


            } else if (CameraInitialization.stoneMiddle) {
                //thing is on middle
                telemetry.addData("Found on the middle", "");
                telemetry.update();
                move(30,.4);



            } else if (CameraInitialization.stoneLeft){
                //thing is on left
                telemetry.addData("Found on the left","");
                telemetry.update();
                move(30,.4);

            } else {
                move(30, .4);
            }
        }
    }


    public void move(double distanceMoving, double speedMoving){
        //in inches
        double whellCircumfrance = 4 * Math.PI; //96
        double wheelMotor = 512; //312
        double ticks = (distanceMoving * (wheelMotor/whellCircumfrance));

        robot.setPower(0,0,0,0);

        robot.rightForwardWheel.setTargetPosition((int)Math.round(ticks));
        robot.rightRearWheel.setTargetPosition((int)Math.round(ticks));
        robot.leftForwardWheel.setTargetPosition((int)Math.round(ticks));
        robot.leftRearWheel.setTargetPosition((int)Math.round(ticks));

        robot.rightForwardWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightRearWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftForwardWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftRearWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.rightRearWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightForwardWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftRearWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftForwardWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        robot.setPower(speedMoving,speedMoving,speedMoving,speedMoving);


        while (opModeIsActive() && (robot.rightRearWheel.isBusy()) && (robot.rightForwardWheel.isBusy()) && (robot.leftForwardWheel.isBusy()) && (robot.leftRearWheel.isBusy())){
            telemetry.addData("Right Target Position",robot.rightForwardWheel.getTargetPosition());
            telemetry.addData("Right Real Position", robot.rightForwardWheel.getCurrentPosition());
            telemetry.addData("Right Back Target Position",robot.rightRearWheel.getTargetPosition());
            telemetry.addData("Right Back Real Position", robot.rightRearWheel.getCurrentPosition());
            telemetry.addData("Left Target Position",robot.leftForwardWheel.getTargetPosition());
            telemetry.addData("Left Real Position", robot.leftForwardWheel.getCurrentPosition());
            telemetry.addData("Left Back Target Position",robot.leftRearWheel.getTargetPosition());
            telemetry.addData("Left Back Real Position", robot.leftRearWheel.getCurrentPosition());
            telemetry.update();
        }
        robot.setPower(0,0,0,0);

        robot.rightForwardWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightRearWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftForwardWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftRearWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}