package org.firstinspires.ftc.teamcode.subsystems.cvpipelines.testing;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Rect;


import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.List;
import java.util.HashMap;

public class PixelGroupDetectionPipeline extends OpenCvPipeline {
    // CONVERSION FACTORS:
    // 179, 255, 255
    // NORMAL H * 0.5
    // NORMAL S * 2.55
//    public static final Scalar upperWhite = new Scalar(202, 41, 255);
//    public static final Scalar lowerWhite = new Scalar(0, 0, 210);

    //idea: have stricter values that don't overlap, then chose the contour that has the most of the stricter values

    //dict key=color, values=array of contours of colors
    //for all the colors:
        //add contours to dict for key=color
    //for all the colors in dictionary
        //look at all the other contours not in dictionary[color], see if overlap
//    public <String, MatOfPoint>
//    HashMap<String,MatOfPoint> colors_to_contours = new HashMap<String, MatOfPoint>();
    // NORMAL V * 2.55
    public int x = 0;
    public int y = 0;
    public int width = 0;
    public int height = 0;
    enum Color {
        WHITE(new Scalar(213, 20, 255), new Scalar(0, 0, 170), true),
        YELLOW(new Scalar(28, 195, 255), new Scalar(12, 82, 230), true),
        PURPLE(new Scalar(208, 90, 255), new Scalar(126, 22, 164), true),
        GREEN(new Scalar(85, 160, 238), new Scalar(19, 0, 157), true);
        public final Scalar UPPER;
        public final Scalar LOWER;
        public final boolean display;
        Color(Scalar upper, Scalar lower, boolean display) {
            this.UPPER = upper; this.LOWER = lower; this.display = display;
        }
    }
    private static final Color[] colors = new Color[]{Color.WHITE, Color.YELLOW, Color.PURPLE, Color.GREEN};
    @Override
    public Mat processFrame(Mat input) {
        Mat output = input.clone();
        Mat hsv = new Mat();
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        Mat kernel = Imgproc.getStructuringElement(Imgproc.CV_SHAPE_RECT, new Size(15, 15));

        ArrayList<Pixel> pixels = new ArrayList<>();
        for (Color c : colors) {
            List<MatOfPoint> contours = new ArrayList<>();
            Mat inRange = new Mat();
            Mat denoisedInRange = new Mat();
            Core.inRange(hsv, c.LOWER, c.UPPER, inRange);

            Imgproc.morphologyEx(inRange, denoisedInRange, Imgproc.MORPH_DILATE, kernel);
            Imgproc.findContours(denoisedInRange, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

            if (c == Color.PURPLE)
            {
                Imgproc.cvtColor(denoisedInRange, output, Imgproc.COLOR_GRAY2RGB);
            }
            for (int i = 0; i < contours.size(); i++) {
                MatOfPoint cont = contours.get(i);
                MatOfPoint2f c2f = new MatOfPoint2f(cont.toArray());
                double peri = Imgproc.arcLength(c2f, true);
                MatOfPoint2f approx = new MatOfPoint2f();
                Imgproc.approxPolyDP(c2f, approx, 0.05 * peri, true);

                Point[] points = approx.toArray();
                if (points.length == 6 && Imgproc.contourArea(cont) > input.width() / 10.0 && true) {
//                if (Imgproc.contourArea(cont) > input.width() / 10.0 && c.display) {
                        pixels.add(new Pixel(cont, c));
//                        Imgproc.drawContours(input, contours, i, new Scalar(255, 255, 0), 2);
//                        Imgproc.putText(input, c.name(), new Point(contours.get(i).get(0, 0)), Imgproc.FONT_HERSHEY_PLAIN, 2, new Scalar(0, 255, 0), 2);
                }
            }

        }
        ArrayList<Pixel> pixels_final = new ArrayList<>();
        int p_idx=0;
        while (p_idx<pixels.size()){
            Pixel p = pixels.get(p_idx);
            Rect p_rect = Imgproc.boundingRect(p.contour);
            //top left of p
            int x1 = p_rect.x;
            int y1 = p_rect.y;
            //bottom right of p
            int x2 = p_rect.x+p_rect.width;
            int y2 = p_rect.y+p_rect.height;

            //area
            double p_area = Imgproc.contourArea(p.contour);
            int p2_idx=0;
            while (p2_idx<pixels.size()){
//            for (int p2_idx=0; p2_idx<pixels.size(); p2_idx++){
                Pixel p2 = pixels.get(p2_idx);
                if (p != p2) {
                    Rect p2_rect = Imgproc.boundingRect(p2.contour);
                    //top left of p2
                    int x3 = p2_rect.x;
//                    int x=x3;
                    int y3 = p2_rect.y;
                    //bottom right of p2
                    int x4 = p2_rect.x+p2_rect.width;
                    int y4 = p2_rect.y+p2_rect.height;

                    // find intersection:
                    int xL = Math.max(x1, x3);
                    int xR = Math.min(x2, x4);
                    if (xR > xL){
                        x=xL;
                        width=xR-xL;
                        int yT = Math.max(y1, y3);
                        int yB = Math.min(y2, y4);
                        if (yB > yT){
                            y=yB;
                            height=yB-yT;
//                            Imgproc.rectangle(input, new Rect(xL, yT, xR-xL, yB-yT), new Scalar(255,0,0), 5);
                            double p2_area = Imgproc.contourArea(p2.contour);

                            if (p_area>p2_area){
                                pixels.remove(pixels.get(p2_idx));
                            }else{
                                pixels.remove(pixels.get(p_idx));
                            }

//                            pixels.remove(p_idx);
//                            Imgproc.rectangle(input, new Point(xL, yB), new Point(xR-xL, yB-yT));
                        }
//                            return null;
//                        else
//                            return new Rect(xL, yB, xR-xL, yB-yT);
                    }


                }
                p2_idx++;
            }
            p_idx++;
        }
        for (Pixel p : pixels){
            List<MatOfPoint> contours = new ArrayList<>();
            contours.add(p.contour);
            Imgproc.drawContours(input, contours, 0, new Scalar(255, 255, 0), 2);
            Imgproc.putText(input, p.color.name(), new Point(contours.get(0).get(0, 0)), Imgproc.FONT_HERSHEY_PLAIN, 2, new Scalar(0, 255, 0), 2);
        }
        return input;
    }
    public static class Pixel {
        public MatOfPoint contour;
        public Color color;
        public Pixel(MatOfPoint contour, Color color) {
            this.contour = contour;
            this.color = color;
        }
    }
}

// ERROR - maybe fixed?
/*
**Pipeline PixelGroupDetectionPipeline is throwing 1 exception(s)**

        > java.lang.IndexOutOfBoundsException: Index 2 out of bounds for length 2
        at java.base/jdk.internal.util.Preconditions.outOfBounds(Preconditions.java:64)
        at java.base/jdk.internal.util.Preconditions.outOfBoundsCheckIndex(Preconditions.java:70)
        at java.base/jdk.internal.util.Preconditions.checkIndex(Preconditions.java:248)
        at java.base/java.util.Objects.checkIndex(Objects.java:372)
        at java.base/java.util.ArrayList.remove(ArrayList.java:536)
        at org.firstinspires.ftc.teamcode.subsystems.cvpipelines.testing.PixelGroupDetectionPipeline.processFrame(PixelGroupDetectionPipeline.java:138)
        at org.openftc.easyopencv.OpenCvPipeline.processFrameInternal(OpenCvPipeline.java:70)
        at org.openftc.easyopencv.ProcessFrameInternalAccessorKt.processFrameInternal(ProcessFrameInternalAccessor.kt:5)
        at com.github.serivesmejia.eocvsim.pipeline.PipelineManager$update$pipelineJob$1.invokeSuspend(PipelineManager.kt:307)
        at kotlin.coroutines.jvm.internal.BaseContinuationImpl.resumeWith(ContinuationImpl.kt:33)
        at kotlinx.coroutines.DispatchedTask.run(DispatchedTask.kt:106)
        at java.base/java.util.concurrent.Executors$RunnableAdapter.call(Executors.java:515)
        at java.base/java.util.concurrent.FutureTask.run(FutureTask.java:264)
        at java.base/java.util.concurrent.ScheduledThreadPoolExecutor$ScheduledFutureTask.run(ScheduledThreadPoolExecutor.java:304)
        at java.base/java.util.concurrent.ThreadPoolExecutor.runWorker(ThreadPoolExecutor.java:1128)
        at java.base/java.util.concurrent.ThreadPoolExecutor$Worker.run(ThreadPoolExecutor.java:628)
        at java.base/java.lang.Thread.run(Thread.java:829)

        ! It has been thrown 1 times, and will expire in 8.4 seconds

 */
