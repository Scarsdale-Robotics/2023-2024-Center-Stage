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

// git push example


// NOTE: changing this code very soon :)
public class PropDetectionPipeline extends OpenCvPipeline {
    Telemetry telemetry;

    public Mat frame;
    public Mat sub;
    public Mat temp = new Mat();

    public int width;
    public int height;

    public static Scalar upperRange = new Scalar(110.5,255,255);  // Range needs fixing
    public static Scalar lowerRange = new Scalar(86,214,64);

    public AtomicBoolean hasStarted = new AtomicBoolean(false);
    public AtomicInteger lateralOffset = new AtomicInteger(0);

    public int contour_dim_ratio;

    public PropDetectionPipeline(Telemetry telemetry){
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {
        this.telemetry = telemetry;
        hasStarted.set(true);
        //open cv defaults to bgr, but we are completely in rgb/hsv, this is because pipeline's input/output rgb
        height = input.height()/3;
        width = input.width();
        int y = 2 * height;
        int x = 0;
        this.sub = input;
//        int posy = height/2;
//        int posx = width / 2;
        MatOfPoint contour = getMaxContour();
        if (contour != null) {
            Imgproc.drawContours(sub, new ArrayList<MatOfPoint>(Arrays.asList(contour)), 0, new Scalar(255, 255, 0), 2);
            Rect b = Imgproc.boundingRect(contour);
            Imgproc.rectangle(sub, b, new Scalar(255, 255, 0));
            Point center = new Point(b.x + b.width/2, b.y + b.height/2);
            Imgproc.circle(sub, center, 2, new Scalar(255, 255, 0));
            telemetry.addData("x",b.x + b.width/2);
            telemetry.addData("y",b.y + b.height/2);
            telemetry.update();
        }
//        telemetry.addData("foo", 0);
        telemetry.addData("height", height);
        telemetry.addData("width", width);
        telemetry.update();
//
//        telemetry.addData("height", height);
//        telemetry.addData("width", width);

//        if(contour != null) {
////            List<MatOfPoint> coneContour = new ArrayList<>();
//            Rect b = Imgproc.boundingRect(contour);
////            contour_dim_ratio=b.width/b.height;
////            if (contour_dim_ratio < 1) {
//            coneContour.add(contour);
//            Imgproc.drawContours(sub, coneContour, 0, new Scalar(255, 255, 0), 2);
//            posx = b.x + b.width / 2;
//            posy = b.y + b.height / 2;
//            Imgproc.circle(sub, new Point(posx, posy), 2, new Scalar(255, 255, 0)); //rgb DRAW CIRCLE
////            }

//        }

//        int offset = posx - (width / 2 + 40);
//        lateralOffset.set(offset);

        return input;
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
