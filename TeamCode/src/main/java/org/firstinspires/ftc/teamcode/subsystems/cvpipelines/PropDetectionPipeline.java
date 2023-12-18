package org.firstinspires.ftc.teamcode.subsystems.cvpipelines;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
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

public class PropDetectionPipeline implements VisionProcessor {
    public Mat frame;
    public Mat sub;
    public Mat temp = new Mat();

    public int width;
    public int height;

//    public static Scalar upperRange = new Scalar(176,114,120);  // Range needs fixing
//    public static Scalar lowerRange = new Scalar(167,66,41);

    public static Scalar blueUpperRange = new Scalar(134,255,255);  // Range needs fixing
    public static Scalar blueLowerRange = new Scalar(86,41,111);

    public AtomicBoolean hasStarted = new AtomicBoolean(false);
    public AtomicInteger lateralOffset = new AtomicInteger(0);
    public static Scalar redUpperRange1 = new Scalar(49,216,255);  // Range needs fixing
    public static Scalar redLowerRange1 = new Scalar(0,134,211);

    public static Scalar redUpperRange2 = new Scalar(22, 255, 199);  // Range needs fixing
    public static Scalar redLowerRange2 = new Scalar(0, 212, 75);

    public Point centerPoint;

    public int place;

    public int x;

    public int y;

    boolean isRedTeam=true;

    public PropDetectionPipeline(boolean isRedTeam){
        this.isRedTeam = isRedTeam;
    }

    public int getPosition(){
        return place;
    }

    public MatOfPoint getMaxContour() {
        Mat hsvmat = new Mat();
        Imgproc.cvtColor(sub, hsvmat, Imgproc.COLOR_RGB2HSV);
//        List<MatOfPoint> contourList = new ArrayList<>();
        List<MatOfPoint> contourList1 = new ArrayList<>();
        List<MatOfPoint> contourList2 = new ArrayList<>();
//        if(isRedTeam) {
////            Scalar upperRange1 = new Scalar(176,114,120);  // Range needs fixing
////            Scalar lowerRange1 = new Scalar(167,66,41);
////            Scalar upperRange2 = new Scalar(179, 161, 170);
////            Scalar lowerRange2 = new Scalar(160, 75, 99);
//            Mat mask1 = new Mat();
//            Mat mask2 = new Mat();
//            Core.inRange(hsvmat, redLowerRange1, redUpperRange1, mask1);
//            Core.inRange(hsvmat, redLowerRange2, redUpperRange2, mask2);
////            Mat mask = mask1 | mask2;
//            Imgproc.findContours(mask1, contourList1, temp, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
//            Imgproc.findContours(mask2, contourList2, temp, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
//
//        }
        if (isRedTeam){
            Mat mask1 = new Mat();
            Mat mask2 = new Mat();
            Core.inRange(hsvmat, redLowerRange1, redUpperRange1, mask1);
            Core.inRange(hsvmat, redLowerRange2, redUpperRange2, mask2);
//            List<MatOfPoint> contourList1 = new ArrayList<>();
//            Mat mask = mask1 | mask2;
            Imgproc.findContours(mask1, contourList1, temp, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            Imgproc.findContours(mask2, contourList2, temp, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        }else {
            Mat mask = new Mat();
            Core.inRange(hsvmat, blueLowerRange, blueUpperRange, mask);
            Imgproc.findContours(mask, contourList1, temp, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        }
        double maxArea1 = Double.MIN_VALUE;
        int best_idx1 = -1;

        double maxArea2 = Double.MIN_VALUE;
        int best_idx2 = -1;

        for (int idx = 0; idx<contourList1.size(); idx++) {
//            Imgproc.drawContours(sub, new ArrayList<MatOfPoint>(Arrays.asList(contourList1.get(idx))), 0, new Scalar(255, 0, 0), 2);
            double area = Imgproc.contourArea(contourList1.get(idx));
            if (area > maxArea1) {
                maxArea1 = area;
                best_idx1 = idx;
            }
        }
        for (int idx = 0; idx<contourList2.size(); idx++) {
//            Imgproc.drawContours(sub, new ArrayList<MatOfPoint>(Arrays.asList(contourList2.get(idx))), 0, new Scalar(0, 255, 0), 2);
            double area = Imgproc.contourArea(contourList2.get(idx));
            if (area > maxArea2) {
                maxArea2 = area;
                best_idx2 = idx;
            }
        }
        if (best_idx1!=-1) {

            Imgproc.drawContours(sub, new ArrayList<MatOfPoint>(Arrays.asList(contourList1.get(best_idx1))), 0, new Scalar(0, 100, 100), 2);

        }
        if (best_idx2!=-1) {

            Imgproc.drawContours(sub, new ArrayList<MatOfPoint>(Arrays.asList(contourList2.get(best_idx2))), 0, new Scalar(100, 100, 0), 2);

        }
//        if (best_idx!=-1) {
//            return contourList2.get(best_idx);
//        }else{
//            return null;
//        }
        return null;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat input, long captureTimeNanos) {
        hasStarted.set(true);
        //open cv defaults to bgr, but we are completely in rgb/hsv, this is because pipeline's input/output rgb
//        height = input.height();
//        width = input.width();
        this.sub = input;
//        MatOfPoint contour = getMaxContour();
//        List<Rect> cropped_rectangles = new Rect(width*2/3, 0,width/3, height);
//        List<MatOfPoint> max_contours = new ArrayList<>(3);
//        double maxArea = Double.MIN_VALUE;
//        int best_idx = 0;
//        MatOfPoint best_contour = new MatOfPoint();
//        Imgproc.rectangle(sub, new Rect(width*2/3,0,width/3, height), new Scalar(255, 255, 0));
//        Imgproc.rectangle(sub, b, new Scalar(255, 255, 0));
//        for (int i=0; i<3; i++){
//            Rect crop = new Rect(width*i/3,0,width/3, height);
//            Imgproc.rectangle(input, crop, new Scalar(255, 255, 0));
//            this.sub = new Mat(input, crop);
//            MatOfPoint contour = getMaxContour();
//
//            if (contour !=null) {
//                double area = Imgproc.contourArea(contour);
//
//                if (area > maxArea) {
////                    best_contour = contour;
//                    maxArea = area;
//                    best_idx = i;
//                }
//            }
//        }
//        Imgproc.drawContours(input, new ArrayList<MatOfPoint>(Arrays.asList(best_contour)), 0, new Scalar(255, 255, 0), 1);
//        telemetry.addData("best idx", best_idx);
//        telemetry.update();
//        return input;
//        return cropped1;

        MatOfPoint contour = getMaxContour();

        if (contour != null) {
            Imgproc.drawContours(sub, new ArrayList<MatOfPoint>(Arrays.asList(contour)), 0, new Scalar(255, 255, 0), 2);
            Rect b = Imgproc.boundingRect(contour);
            Imgproc.rectangle(sub, b, new Scalar(255, 255, 0));
            x = b.x + b.width/2;
            y = b.y + b.height/2;
            Point centerPoint = new Point(x, y);
            Imgproc.circle(sub, centerPoint, 2, new Scalar(255, 255, 0));
            if (x<=150) {
                place=0;
            }
            else if (x>150 && x<500){
                place=1;
            }
            else{
                place=2;
            }
        }

        return input;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}
