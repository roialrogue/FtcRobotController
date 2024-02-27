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
    public DcMotor BeltMotor;
    //"EM0"
    public DcMotor AirplaneMotor;
    //"EM2"
    public DcMotor HangMotor;
    //"EM3"
    public Servo LeftInTake;
    //"CS0"
    public Servo RightInTake;
    //"CS2"
    public Servo ClawUpDown;
    //"ES0"
    public Servo ClawLeftRight;
    //"ES2"
    public Servo AirplaneServo;
    //"ES4"
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

            leftForwardWheel = hwMap.get(DcMotor.class, "CM0");
            leftForwardWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftForwardWheel.setDirection(DcMotorSimple.Direction.REVERSE);
            leftForwardWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftForwardWheel.setPower(0);

            leftRearWheel = hwMap.get(DcMotor.class, "CM1");
            leftRearWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftRearWheel.setDirection(DcMotorSimple.Direction.FORWARD);
            leftRearWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftRearWheel.setPower(0);

            rightForwardWheel = hwMap.get(DcMotor.class, "CM2");
            rightForwardWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightForwardWheel.setDirection(DcMotor.Direction.FORWARD);
            rightForwardWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightForwardWheel.setPower(0);

            rightRearWheel = hwMap.get(DcMotor.class, "CM3");
            rightRearWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightRearWheel.setDirection(DcMotor.Direction.REVERSE);
            rightRearWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightRearWheel.setPower(0);

            HangMotor = hwMap.get(DcMotor.class, "EM3");
            HangMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            HangMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            HangMotor.setPower(0);

            BeltMotor = hwMap.get(DcMotor.class, "EM0");
            BeltMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            BeltMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BeltMotor.setPower(0);

            AirplaneMotor = hwMap.get(DcMotor.class, "EM2");
            AirplaneMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            AirplaneMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            AirplaneMotor.setPower(0);

            AirplaneServo = hwMap.get(Servo.class, "ES4");

            LeftInTake = hwMap.get(Servo.class, "CS0");

            RightInTake = hwMap.get(Servo.class, "CS2");

            ClawLeftRight = hwMap.get(Servo.class, "ES2");

            ClawUpDown = hwMap.get(Servo.class, "ES0");

        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        WebcamName webcamName = hwMap.get(WebcamName.class, "Webcam 1");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);



    }

    public void closeRight() {RightInTake.setPosition(.50);}
    public void closeLeft() {LeftInTake.setPosition(.70);}
    public void openRight() {RightInTake.setPosition(.18);}
    public void openLeft() {LeftInTake.setPosition(.95);}
    public void wristUp() {ClawUpDown.setPosition(.7);}
    public void wristDown() {ClawUpDown.setPosition(.52);}
    public void rotateUp() {ClawLeftRight.setPosition(.05);}
    public void rotateDown() {ClawLeftRight.setPosition(.725);}

    public  void slidesTo(int slidePos, double power){
        BeltMotor.setTargetPosition(slidePos);
        BeltMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BeltMotor.setPower(power);
    }
    public void slidesTo(int slidePos) {
        slidesTo(slidePos,1.0);
    }


}

