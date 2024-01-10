package org.firstinspires.ftc.teamcode;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class GPTCamera extends OpenCvPipeline {
    private Mat workingMatrix = new Mat();
    public static boolean leftSide = false;
    public static boolean rightSide = false;
    public static boolean middleSide = false;
    public static boolean nonSide = false;


    public static int matBrowStart = 0;
    public static int matBrowEnd = 426;
    public static int matBcolStart = 0;
    public static int matBcolEnd = 720;

    public static int matCrowStart = 426;
    public static int matCrowEnd = 852;
    public static int matCcolStart = 0;
    public static int matCcolEnd = 720;

    public static int matArowStart = 852;
    public static int matArowEnd = 1278;
    public static int matAcolStart = 0;
    public static int matAcolEnd = 720;

    boolean isBlue;

    public GPTCamera(boolean isBlue) {
        this.isBlue = isBlue;
    }

    Mat matA = workingMatrix.submat(matArowStart, matArowEnd, matAcolStart, matAcolEnd);
    Mat matB = workingMatrix.submat(matBrowStart, matBrowEnd, matBcolStart, matBcolEnd);
    Mat matC = workingMatrix.submat(matCrowStart, matCrowEnd, matCcolStart, matCcolEnd);
    public final Mat processFrame(Mat input) {
        input.copyTo(workingMatrix);
        if (workingMatrix.empty()) {
            return input;
        }
        Imgproc.cvtColor(workingMatrix, workingMatrix, Imgproc.COLOR_RGB2HSV);

        Scalar lowVal, highVal;
        if (isBlue) {
            lowVal = new Scalar(150, 50, 50);
            highVal = new Scalar(290, 255, 255);
        } else {
            lowVal = new Scalar(0, 100, 100);
            highVal = new Scalar(11, 255, 255);
        }
        Mat left = workingMatrix.submat(matArowStart, matArowEnd, matAcolStart, matAcolEnd);
        Mat middle = workingMatrix.submat(matBrowStart, matBrowEnd, matBcolStart, matBcolEnd);
        Mat right = workingMatrix.submat(matCrowStart, matCrowEnd, matCcolStart, matCcolEnd);

        double leftValue = Core.sumElems(left).val[0];
        double middleValue = Core.sumElems(middle).val[0];
        double rightValue = Core.sumElems(right).val[0];

        left.release();
        middle.release();
        right.release();

        double leftper = Math.round(leftValue * 100);
        double middleper = Math.round(middleValue * 100);
        double rightper = Math.round(rightValue * 100);



        if (leftper > rightper && leftper > middleper) {
            leftSide = true;
        } else if (middleper > leftper && middleper > rightper){
            middleSide = true;
        } else if (rightper > leftper && rightper > middleper){
            rightSide = true;
        } else {
            nonSide = true;
        }

        Imgproc.cvtColor(workingMatrix, workingMatrix, Imgproc.COLOR_GRAY2RGB);
        return workingMatrix;
    }
}
