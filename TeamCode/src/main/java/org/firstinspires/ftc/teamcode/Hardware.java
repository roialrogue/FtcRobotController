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
    public DcMotor AMotorUpDown;
    //"CM4"
    public DcMotor AMotorOutIn;
    //"CM5"
    public DcMotor AMotorIntake;
    //"CM6"
    public Servo PServo1;

    public Servo PServo2;

    public Servo AServo;

    public static double maxSpeed = 0.8;


    public BNO055IMU gyro;

    public RevColorSensorV3 color;

    public OpenCvCamera camera;
    private static Hardware myInstance = null;

    public static Hardware getInstance() {
        if (myInstance == null) {
            myInstance = new Hardware();
        }
        return myInstance;
    }

    public void init(HardwareMap hwMap) {


        try {
            rightForwardWheel = hwMap.get(DcMotor.class, "CM2");
            rightForwardWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightForwardWheel.setDirection(DcMotor.Direction.REVERSE);
            rightForwardWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightForwardWheel.setPower(0);
        } catch (Exception p_exception) {
            rightForwardWheel = null;
        }

        try {
            leftForwardWheel = hwMap.get(DcMotor.class, "CM0");
            leftForwardWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftForwardWheel.setDirection(DcMotorSimple.Direction.FORWARD);
            leftForwardWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftForwardWheel.setPower(0);
        } catch (Exception p_exception) {
            leftForwardWheel = null;
        }

        try {
            rightRearWheel = hwMap.get(DcMotor.class, "CM3");
            rightRearWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightRearWheel.setDirection(DcMotor.Direction.REVERSE);
            rightRearWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightRearWheel.setPower(0);
        } catch (Exception p_exception) {
            rightRearWheel = null;
        }

        try {
            leftRearWheel = hwMap.get(DcMotor.class, "CM1");
            leftRearWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftRearWheel.setDirection(DcMotorSimple.Direction.FORWARD);
            leftRearWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftRearWheel.setPower(0);
        } catch (Exception p_exception) {
            leftRearWheel = null;
        }

        try {
            AMotorUpDown = hwMap.get(DcMotor.class, "CM4");
            AMotorUpDown.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            AMotorUpDown.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            AMotorUpDown.setPower(0);
        } catch (Exception p_exception) {
            AMotorUpDown = null;
        }

        try {
            AMotorOutIn = hwMap.get(DcMotor.class, "CM5");
            AMotorOutIn.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            AMotorOutIn.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            AMotorOutIn.setPower(0);
        } catch (Exception p_exception) {
            AMotorOutIn = null;
        }

        try {
            AMotorIntake = hwMap.get(DcMotor.class, "CM6");
            AMotorIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            AMotorIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            AMotorIntake.setPower(0);
        } catch (Exception p_exception) {
            AMotorIntake = null;
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

    public void setPower(double fr, double br, double fl, double bl) {
        if (rightForwardWheel != null) {
            rightForwardWheel.setPower(Range.clip(fr, -maxSpeed, maxSpeed));
        }
        if (rightRearWheel != null) {
            rightRearWheel.setPower(Range.clip(br, -maxSpeed, maxSpeed));
        }
        if (leftForwardWheel != null) {
            leftForwardWheel.setPower(Range.clip(fl, -maxSpeed, maxSpeed));
        }
        if (leftRearWheel != null) {
            leftRearWheel.setPower(Range.clip(bl, -maxSpeed, maxSpeed));
        }


    /*public void setPower(double rightFront, double leftFront, double rightBack, double leftBack) {
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


    }*/


    }
}

