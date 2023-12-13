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
        CameraInitialization cameraPipeline = new CameraInitialization(telemetry, true);
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
        /*runtime.reset();
        while(runtime.seconds() < 5) {
            hw.leftForwardWheel.setPower(1);
        }
        hw.leftForwardWheel.setPower(0);*/

        hw.camera.stopStreaming();



            if (cameraWorked[0]) {
                //runs if camera works
                CameraInitialization.Location location = cameraPipeline.getLocation();
                if (CameraInitialization.stoneRight) {
                    //thing is on right
                    telemetry.addData("Found on the right", "");
                    telemetry.update();
                    move(65,.4);



                } else if (CameraInitialization.stoneMiddle) {
                    //thing is on middle
                    telemetry.addData("Found on the middle", "");
                    //telemetry.update();
                    move(65,.4);



                } else if (CameraInitialization.stoneLeft) {
                    //thing is on left
                    telemetry.addData("Found on the left", "");
                    telemetry.update();
                    move(65,.4);
                } else {
                    move(65, .4);
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

        while (opModeIsActive() && (robot.rightForwardWheel.getCurrentPosition() + 10 < ticks || robot.rightForwardWheel.getCurrentPosition() - 10 > ticks)){

        }
        robot.setPower(0,0,0,0);

        robot.rightForwardWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightRearWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftForwardWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftRearWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
