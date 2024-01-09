package org.firstinspires.ftc.teamcode;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class GPTCamera extends OpenCvPipeline {
    public enum Location {
        LEFT,
        MIDDLE,
        RIGHT,
        NOT_FOUND
    }

    private Location location;

    public static boolean leftSide = false;
    public static boolean rightSide = false;
    public static boolean middleSide = false;
    public static boolean nonSide = false;


    static final Rect LEFT_ROI = new Rect(new Point(0, 0), new Point(426, 720));
    static final Rect MIDDLE_ROI = new Rect(new Point(426, 0), new Point(852, 720));
    static final Rect RIGHT_ROI = new Rect(new Point(852, 0), new Point(1278, 720));

    boolean isBlue;

    public GPTCamera(boolean isBlue) {
        this.isBlue = isBlue;
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2HSV);

        Scalar lowVal, highVal;
        if (isBlue) {
            lowVal = new Scalar(150, 50, 50);
            highVal = new Scalar(290, 255, 255);
        } else {
            lowVal = new Scalar(0, 100, 100);
            highVal = new Scalar(11, 255, 255);
        }
        Mat left = input.submat(LEFT_ROI);
        Mat middle = input.submat(MIDDLE_ROI);
        Mat right = input.submat(RIGHT_ROI);

        double leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255;
        double middleValue = Core.sumElems(middle).val[0] / MIDDLE_ROI.area() / 255;
        double rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255;

        left.release();
        middle.release();
        right.release();

        double leftper = Math.round(leftValue * 100);
        double middleper = Math.round(middleValue * 100);
        double rightper = Math.round(rightValue * 100);



        if (leftper > rightper && leftper > middleper) {
            leftSide = true;
            /*.addData("Found on the", "right");*/

        } else if (middleper > leftper && middleper > rightper){
            middleSide = true;
            /*telemetry.addData("Found on the", "middle");*/
        } else if (rightper > leftper && rightper > middleper){
            rightSide = true;
            /*telemetry.addData("Found on the", "left");*/
        } else {
            nonSide = true;
            /*telemetry.addData("Did not find","Nooo");
            telemetry.update();*/
        }

        Imgproc.cvtColor(input, input, Imgproc.COLOR_GRAY2RGB);
        return input;
    }

    /*public static Location getLocation() {
        //return location;
        return */

}
