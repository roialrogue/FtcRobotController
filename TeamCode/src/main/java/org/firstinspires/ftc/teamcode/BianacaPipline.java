/*
package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class BianacaPipline extends OpenCvPipeline {
    private Mat workingMatrix = new Mat();
    public String position = "AssumingUpperTray";
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

    public static boolean leftSide = false;
    public static boolean rightSide = false;
    public static boolean middleSide = false;

    public BianacaPipline() {

    }
    public final Mat processFrame(Mat input) {
        input.copyTo(workingMatrix);

        if (workingMatrix.empty()) {
            return input;
        }

        Mat matA = workingMatrix.submat(matArowStart, matArowEnd, matAcolStart, matAcolEnd);
        Mat matB = workingMatrix.submat(matBrowStart, matBrowEnd, matBcolStart, matBcolEnd);
        Mat matC = workingMatrix.submat(matCrowStart, matCrowEnd, matCcolStart, matCcolEnd);

        Imgproc.rectangle(workingMatrix, new Rect( matAcolStart, matArowStart, (matArowEnd - matArowStart), (matAcolEnd - matAcolStart)), new Scalar(0, 255, 0));
        Imgproc.rectangle(workingMatrix, new Rect(matBcolStart, matBrowStart, (matBrowEnd - matBrowStart), (matBcolEnd - matBcolStart)), new Scalar(0, 255, 0));
        Imgproc.rectangle(workingMatrix, new Rect(matCcolStart, matCrowStart, (matCrowEnd - matCrowStart), (matCcolEnd - matCcolStart)), new Scalar(0, 255, 0));



        totalA = Core.sumElems(matA).val[0];
        totalA /= matA.rows() * matA.cols();
        totalB = Core.sumElems(matA).val[1];
        totalB /= matA.rows() * matA.cols();
        totalC = Core.sumElems(matA).val[2];
        totalC /= matA.rows() * matA.cols();

        Atotal = (totalA + totalB + totalC);

        if ((totalA > totalB) && (totalA > totalC)) {
            leftSide = true;
        } else if ((totalC > totalA) && (totalC > totalB)) {
            middleSide = true;
        } else {
            rightSide = true;
        }

        return workingMatrix;
    }
}
*/
