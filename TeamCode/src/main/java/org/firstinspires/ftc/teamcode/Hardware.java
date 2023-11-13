package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

public class Hardware {

    public DcMotor rightForwardWheel;
    //"CM3"
    public DcMotor leftForwardWheel;
    //"CM1"
    public DcMotor rightRearWheel;
    //"CM2"
    public DcMotor leftRearWheel;
    //"CM0"
    public DcMotor AMotor1;
    //"CM5"
    public Servo PServo1;

    public Servo PServo2;

    public Servo AServo;


    public BNO055IMU gyro;

    public RevColorSensorV3 color;

    public static double maxSpeed = 0.8;

    public OpenCvCamera camera;
    private static Hardware myInstance = null;

    public static Hardware getInstance() {
        if (myInstance == null) {
            myInstance = new Hardware();
        }
        return myInstance;
    }

    public void init(HardwareMap hwMap) {

        try{
            rightForwardWheel = hwMap.get(DcMotor.class, "CM3");
            rightForwardWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightForwardWheel.setDirection(DcMotorSimple.Direction.REVERSE);
            rightForwardWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightForwardWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightForwardWheel.setPower(0);
        } catch (Exception p_exception) {
            rightForwardWheel = null;
        }

        try{
            leftForwardWheel = hwMap.get(DcMotor.class, "CM1");
            leftForwardWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftForwardWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftForwardWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftForwardWheel.setPower(0);
        } catch (Exception p_exception) {
            leftForwardWheel = null;
        }

        try{
            rightRearWheel = hwMap.get(DcMotor.class, "CM2");
            rightRearWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightRearWheel.setDirection(DcMotorSimple.Direction.REVERSE);
            rightRearWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightRearWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightRearWheel.setPower(0);
        } catch (Exception p_exception) {
            rightRearWheel = null;
        }

        try{
            leftRearWheel = hwMap.get(DcMotor.class, "CM0");
            leftRearWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftRearWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftRearWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftRearWheel.setPower(0);
        } catch (Exception p_exception) {
            leftRearWheel = null;
        }

        try{
            AMotor1 = hwMap.get(DcMotor.class, "CM5");
            AMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            AMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            AMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            AMotor1.setPower(0);
        } catch (Exception p_exception) {
            AMotor1 = null;
        }

        try {
            PServo1 = hwMap.get(Servo.class, "CS1");
        } catch (Exception p_exception) {
            PServo1 = null;
        }

        try {
            PServo2 = hwMap.get(Servo.class, "CS2");
        } catch (Exception p_exception) {
            PServo2 = null;
        }

        try {
            AServo = hwMap.get(Servo.class, "CS0");
        } catch (Exception p_exception) {
            AServo = null;
        }


        try {
            gyro = hwMap.get(BNO055IMU.class, "gyro");
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.loggingEnabled = true;
            parameters.loggingTag = "gyro";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
            gyro.initialize(parameters);
        } catch (Exception p_exception) {
            gyro = null;
        }

        try {
            color = hwMap.get(RevColorSensorV3.class, "Color");
        } catch (Exception p_exception) {
            color = null;
        }


            int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
            WebcamName webcamName = hwMap.get(WebcamName.class, "Webcam 1");
            camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);


    }

    public void setPower(double rightFront, double leftFront, double rightBack, double leftBack) {
        if (rightForwardWheel != null) {
           rightForwardWheel.setPower(Range.clip(rightFront, -maxSpeed, maxSpeed));
        }
        if (leftForwardWheel != null) {
            leftForwardWheel.setPower(Range.clip(leftFront, -maxSpeed, maxSpeed));
        }
        if (rightRearWheel != null) {
            rightRearWheel.setPower(Range.clip(rightBack, -maxSpeed, maxSpeed));
        }
        if (leftRearWheel != null) {
            leftRearWheel.setPower(Range.clip(leftBack, -maxSpeed, maxSpeed));
        }


    }


}
