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
    public double totalA = 0;
    public double totalB = 0;
    public double totalC = 0;
    public double Atotal = 0;
    public double totalAB = 0;
    public double totalBB = 0;
    public double totalCB = 0;
    public double Btotal = 0;
    public double totalAC = 0;
    public double totalBC = 0;
    public double totalCC = 0;
    public double Ctotal = 0;

    public static int matArowStart = 0;
    public static int matArowEnd = 720; //720
    public static int matAcolStart = 0;
    public static int matAcolEnd = 426; //426
    public static int matBrowStart = 0;
    public static int matBrowEnd = 720; //720
    public static int matBcolStart = 426; //426
    public static int matBcolEnd = 852; //852
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
    public final Mat processFrame(Mat workingMatrix) {

        if (workingMatrix.empty()) {
            return workingMatrix;
        }

        Mat left = workingMatrix.submat(matArowStart,matArowEnd, matAcolStart, matAcolEnd);
        Mat middle = workingMatrix.submat(matBrowStart, matBrowEnd, matBcolStart, matBcolEnd);
        Mat right = workingMatrix.submat(matCrowStart, matCrowEnd, matCcolStart, matCcolEnd);

        Imgproc.rectangle(workingMatrix, new Rect(matAcolStart, matArowStart, (matAcolEnd - matAcolStart), (matArowEnd - matArowStart)), new Scalar(0, 255, 0));
        Imgproc.rectangle(workingMatrix, new Rect(matBcolStart,matBrowStart , (matBcolEnd - matBcolStart), (matBrowEnd - matBrowStart)), new Scalar(0, 0, 0));
        Imgproc.rectangle(workingMatrix, new Rect(matCcolStart, matCrowStart, (matCcolEnd - matCcolStart), (matCrowEnd - matCrowStart)), new Scalar(0, 0, 255));

        if (isBlue) {
            totalA = Core.sumElems(left).val[2];
            totalA -= Core.sumElems(left).val[1];
            totalA -= Core.sumElems(left).val[0];
            totalA /= left.rows() * left.cols();
            totalB = Core.sumElems(middle).val[2];
            totalB -= Core.sumElems(middle).val[1];
            totalB -= Core.sumElems(middle).val[0];
            totalB /= middle.rows() * middle.cols();
            totalC = Core.sumElems(right).val[2];
            totalC -= Core.sumElems(right).val[1];
            totalC -= Core.sumElems(right).val[0];
            totalC /= right.rows() * right.cols();
        } else {
            totalA = Core.sumElems(left).val[0];
            totalA -= Core.sumElems(left).val[1];
            totalA -= Core.sumElems(left).val[2];
            totalA /= left.rows() * left.cols();
            totalB = Core.sumElems(middle).val[0];
            totalB -= Core.sumElems(middle).val[1];
            totalB -= Core.sumElems(middle).val[2];
            totalB /= middle.rows() * middle.cols();
            totalC = Core.sumElems(right).val[0];
            totalC -= Core.sumElems(right).val[1];
            totalC -= Core.sumElems(right).val[2];
            totalC /= right.rows() * right.cols();
        }

        telemetry.addData("Total A",totalA);
        telemetry.addData("Total B",totalB);
        telemetry.addData("Total C",totalC);
        telemetry.update();

         leftSide = false;
         rightSide = false;
         middleSide = false;
         nonSide = false;

        if ((totalA > totalB) && (totalA > totalC)) {
            telemetry.addData("Found on the","right");
            rightSide = true;
        } else if ((totalB > totalA) && (totalB > totalC)){
            telemetry.addData("Found on the","middle");
            middleSide = true;
        } else if ((totalC > totalA) && (totalC > totalB)){
            telemetry.addData("Found on the","left");
            leftSide = true;
        } else {
            telemetry.addData("Found on the","non-side");
            nonSide = true;
        }

        return workingMatrix;
    }
}