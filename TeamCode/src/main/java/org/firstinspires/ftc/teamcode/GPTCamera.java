package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class GPTCamera extends OpenCvPipeline {
    private Mat workingMatrix = new Mat();
    public static boolean leftSide;
    public static boolean rightSide;
    public static boolean middleSide;
    public static boolean nonSide;
    public static int matArowStart = 0;
    public static int matArowEnd = 720;
    public static int matAcolStart = 0;
    public static int matAcolEnd = 426;
    public static int matBrowStart = 0;
    public static int matBrowEnd = 720;
    public static int matBcolStart = 426;
    public static int matBcolEnd = 852;
    public static int matCrowStart = 0;
    public static int matCrowEnd = 720;
    public static int matCcolStart = 852;
    public static int matCcolEnd = 1280;
    boolean isBlue;

    Telemetry telemetry;

    public GPTCamera(boolean isBlue, Telemetry telemetry) {
        this.isBlue = isBlue;
        this.telemetry = telemetry;
    }
    public final Mat processFrame(Mat input) {

        if (input.empty()) {
            return input;
        }

        Imgproc.cvtColor(input, workingMatrix, Imgproc.COLOR_RGB2HSV);

        Scalar lowVal, highVal;
        if(isBlue){
            lowVal = new Scalar(90, 150, 0);
            highVal = new Scalar(140, 255, 255);
        }else{
            lowVal = new Scalar(0, 100, 100);
            highVal = new Scalar(20, 255, 255);
        }
        Core.inRange(workingMatrix, lowVal, highVal, workingMatrix);

        Mat left = workingMatrix.submat(matArowStart,matArowEnd, matAcolStart, matAcolEnd);
        Mat middle = workingMatrix.submat(matBrowStart, matBrowEnd, matBcolStart, matBcolEnd);
        Mat right = workingMatrix.submat(matCrowStart, matCrowEnd, matCcolStart, matCcolEnd);

        Imgproc.rectangle(workingMatrix, new Rect(matAcolStart, matArowStart, (matAcolEnd - matAcolStart), (matArowEnd - matArowStart)), new Scalar(0, 255, 0));
        Imgproc.rectangle(workingMatrix, new Rect(matBcolStart,matBrowStart , (matBcolEnd - matBcolStart), (matBrowEnd - matBrowStart)), new Scalar(0, 0, 0));
        Imgproc.rectangle(workingMatrix, new Rect(matCcolStart, matCrowStart, (matCcolEnd - matCcolStart), (matCrowEnd - matCrowStart)), new Scalar(0, 0, 255));

        double leftValue = Core.sumElems(left).val[0] / (left.rows() * left.cols());
        double middleValue = Core.sumElems(middle).val[0] / (middle.rows() * middle.cols());
        double rightValue = Core.sumElems(right).val[0] / (right.rows() * right.cols());

         leftSide = false;
         rightSide = false;
         middleSide = false;
         nonSide = false;

        if ((leftValue > middleValue) && (leftValue > rightValue)) {
            telemetry.addData("Found on the","left");
            leftSide = true;
        } else if ((middleValue > leftValue) && (middleValue > rightValue)){
            telemetry.addData("Found on the","middle");
            middleSide = true;
        } else if ((rightValue > leftValue) && (rightValue > middleValue)){
            telemetry.addData("Found on the","right");
            rightSide = true;
        } else {
            telemetry.addData("Found on the","non-side");
            nonSide = true;
        }
        telemetry.update();
        return workingMatrix;
    }
}