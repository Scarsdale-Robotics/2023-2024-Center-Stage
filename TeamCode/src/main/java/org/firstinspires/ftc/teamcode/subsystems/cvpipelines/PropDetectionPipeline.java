package org.firstinspires.ftc.teamcode.subsystems.cvpipelines;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;
import java.util.Arrays;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import org.firstinspires.ftc.robotcore.external.Telemetry;


// NOTE: changing this code very soon :)
public class PropDetectionPipeline extends OpenCvPipeline {
    Telemetry telemetry;

    public Mat frame;
    public Mat sub;
    public Mat temp = new Mat();

    public int width;
    public int height;

    public static Scalar upperRange = new Scalar(176,114,120);  // Range needs fixing
    public static Scalar lowerRange = new Scalar(167,66,41);

    public AtomicBoolean hasStarted = new AtomicBoolean(false);
    public AtomicInteger lateralOffset = new AtomicInteger(0);

    public Point centerPoint;

    public int place;

    public int x;

    public int y;


    public PropDetectionPipeline(Telemetry telemetry){
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {
        this.telemetry = telemetry;
        hasStarted.set(true);
        //open cv defaults to bgr, but we are completely in rgb/hsv, this is because pipeline's input/output rgb
        height = input.height();
        width = input.width();
        this.sub = input;
//        MatOfPoint contour = getMaxContour();
//        List<Rect> cropped_rectangles = new Rect(width*2/3, 0,width/3, height);
//        List<MatOfPoint> max_contours = new ArrayList<>(3);
        double maxArea = Double.MIN_VALUE;
        int best_idx = 0;
//        MatOfPoint best_contour = new MatOfPoint();
//        Imgproc.rectangle(sub, new Rect(width*2/3,0,width/3, height), new Scalar(255, 255, 0));
//        Imgproc.rectangle(sub, b, new Scalar(255, 255, 0));
        for (int i=0; i<3; i++){
            Rect crop = new Rect(width*i/3,0,width/3, height);
            Imgproc.rectangle(input, crop, new Scalar(255, 255, 0));
            this.sub = new Mat(input, crop);
            MatOfPoint contour = getMaxContour();

            if (contour !=null) {
                double area = Imgproc.contourArea(contour);

                if (area > maxArea) {
//                    best_contour = contour;
                    maxArea = area;
                    best_idx = i;
                }
            }
        }
//        Imgproc.drawContours(input, new ArrayList<MatOfPoint>(Arrays.asList(best_contour)), 0, new Scalar(255, 255, 0), 1);
        telemetry.addData("best idx", best_idx);
        telemetry.update();
        return input;
//        return cropped1;
//        for
//        if (contour != null) {
//            Imgproc.drawContours(sub, new ArrayList<MatOfPoint>(Arrays.asList(contour)), 0, new Scalar(255, 255, 0), 2);
//            Rect b = Imgproc.boundingRect(contour);
//            Imgproc.rectangle(sub, b, new Scalar(255, 255, 0));
//            x = b.x + b.width/2;
//            y = b.y + b.height/2;
//            Point centerPoint = new Point(x, y);
//            Imgproc.circle(sub, centerPoint, 2, new Scalar(255, 255, 0));
//            if (x<=150) {
//                place=0;
//            }
//            else if (x>150 && x<500){
//                place=1;
//            }
//            else{
//                place=2;
//            }
//            telemetry.addData("x",b.x + b.width/2);
//            telemetry.addData("y",b.y + b.height/2);
//            telemetry.addData("place",place);
//            telemetry.update();
//        }
//        telemetry.addData("height", height);
//        telemetry.addData("width", width);
//        telemetry.update();
//
//        return input;
    }

    public MatOfPoint getMaxContour() {
        Mat hsvmat = new Mat();
        Imgproc.cvtColor(sub, hsvmat, Imgproc.COLOR_RGB2HSV);

        Mat inRange = new Mat();
        Core.inRange(hsvmat, lowerRange, upperRange, inRange);

        List<MatOfPoint> contourList = new ArrayList<>();
        Imgproc.findContours(inRange, contourList, temp, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        double maxArea = Double.MIN_VALUE;
        int best_idx = -1;
        for (int idx = 0; idx<contourList.size(); idx++) {
            double area = Imgproc.contourArea(contourList.get(idx));
            if (area > maxArea) {
                maxArea = area;
                best_idx = idx;
            }
        }
        if (best_idx!=-1) {
            return contourList.get(best_idx);
        }else{
            return null;
        }
    }
}
