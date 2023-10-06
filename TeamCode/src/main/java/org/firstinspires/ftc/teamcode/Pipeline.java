package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class Pipeline extends OpenCvPipeline {

    private Mat workingMatrix = new Mat();

    public int location = 0;

    public String position = "start";

    public double totalA = 0;
    public double totalB = 0;
    public double totalC = 0;
    public double total = 0;

    public static int matArowStart = 0;
    public static int matArowEnd = 480;
    public static int matAcolumnStart = 0;
    public static int matAcolumnEnd = 640;


    public Pipeline() {

    }

    public final Mat processFrame(Mat input) {

        input.copyTo(workingMatrix);

        if (workingMatrix.empty()) {
            return input;
        }

        Mat matA = workingMatrix.submat(matArowStart, matArowEnd, matAcolumnStart, matAcolumnEnd);

        Imgproc.rectangle(workingMatrix, new Rect(matAcolumnStart, matArowStart, (matArowEnd - matArowStart), (matAcolumnEnd - matAcolumnStart)), new Scalar(0, 255, 0));

        //Average pixels color for each color or something
        totalA = Core.sumElems(matA).val[0];
        totalA /= matA.rows() * matA.cols();
        totalB = Core.sumElems(matA).val[1];
        totalB /= matA.rows() * matA.cols();
        totalC = Core.sumElems(matA).val[2];
        totalC /= matA.rows() * matA.cols();

        total = (totalA + totalB + totalC) / 3;

        if((totalA > totalB) && (totalA > totalC)) {
            location = 1;
            position = "location 1";
        } else if ((totalB > totalA) && (totalB > totalC)) {
            location = 2;
            position = "location 2";
        } else {
            location = 3;
            position = "location 3";
        }

        return workingMatrix;


    }



}
