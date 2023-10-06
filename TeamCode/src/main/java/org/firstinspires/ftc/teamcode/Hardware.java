package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.openftc.easyopencv.OpenCvCamera;

public class Hardware {

    public DcMotor rightForwardWheel;

    public DcMotor leftForwardWheel;

    public DcMotor rightRearWheel;

    public DcMotor leftRearWheel;

    public DcMotor demoMotor;

    public Servo servo1;

    public BNO055IMU gyro;

    public OpenCvCamera

    public RevColorSensorV3 color;

    public static double maxSpeed = 1;

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
            rightForwardWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightForwardWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightForwardWheel.setPower(0);
        } catch (Exception p_exception) {
            rightForwardWheel = null;
        }

        try{
            leftForwardWheel = hwMap.get(DcMotor.class, "CM1");
            leftForwardWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftForwardWheel.setDirection(DcMotorSimple.Direction.REVERSE);
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
            demoMotor = hwMap.get(DcMotor.class, "demoMotor");
            demoMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            demoMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            demoMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            demoMotor.setPower(0);
        } catch (Exception p_exception) {
            demoMotor = null;
        }

        try {
            servo1 = hwMap.get(Servo.class, "Servo1");
        } catch (Exception p_exception) {
            servo1 = null;
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
