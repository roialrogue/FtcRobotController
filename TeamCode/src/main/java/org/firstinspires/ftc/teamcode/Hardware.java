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
    public Servo HangServo;

    public Servo InTakeServo1;

    public Servo InTakeServo2;

    public Servo ClawRotationServo;

    public Servo ClawDropServo;

    public Servo AirplaneServo;

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



            rightForwardWheel = hwMap.get(DcMotor.class, "CM2");
            rightForwardWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightForwardWheel.setDirection(DcMotor.Direction.FORWARD);
            rightForwardWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightForwardWheel.setPower(0);


            leftForwardWheel = hwMap.get(DcMotor.class, "CM0");
            leftForwardWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftForwardWheel.setDirection(DcMotorSimple.Direction.REVERSE);
            leftForwardWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftForwardWheel.setPower(0);

            rightRearWheel = hwMap.get(DcMotor.class, "CM3");
            rightRearWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightRearWheel.setDirection(DcMotor.Direction.FORWARD);
            rightRearWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightRearWheel.setPower(0);

            leftRearWheel = hwMap.get(DcMotor.class, "CM1");
            leftRearWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftRearWheel.setDirection(DcMotorSimple.Direction.REVERSE);
            leftRearWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftRearWheel.setPower(0);

            AMotorUpDown = hwMap.get(DcMotor.class, "EH0");
            AMotorUpDown.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            AMotorUpDown.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            AMotorUpDown.setPower(0);

            AMotorOutIn = hwMap.get(DcMotor.class, "EH3");
            AMotorOutIn.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            AMotorOutIn.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            AMotorOutIn.setPower(0);

            HangServo = hwMap.get(Servo.class, "CS0");

            InTakeServo1 = hwMap.get(Servo.class, "CS4");

            InTakeServo2 = hwMap.get(Servo.class, "ES0");

            ClawRotationServo = hwMap.get(Servo.class, "CS2");

            ClawDropServo = hwMap.get(Servo.class, "ES2");

            AirplaneServo = hwMap.get(Servo.class, "ES4");

//            gyro = hwMap.get(BNO055IMU.class, "gyro");
//            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//            parameters.loggingEnabled = true;
//            parameters.loggingTag = "gyro";
//            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
//            gyro.initialize(parameters);

//['            color = hwMap.get(RevColorSensorV3.class, "Color");']


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


    }
}

