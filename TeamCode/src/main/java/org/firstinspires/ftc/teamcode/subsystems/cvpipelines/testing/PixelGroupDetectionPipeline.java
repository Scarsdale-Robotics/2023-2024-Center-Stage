//package org.firstinspires.ftc.teamcode.subsystems.cvpipelines.testing;
//
//import static org.opencv.imgproc.Imgproc.putText;
//
//import org.opencv.core.Core;
//import org.opencv.core.Mat;
//import org.opencv.core.MatOfPoint;
//import org.opencv.core.MatOfPoint2f;
//import org.opencv.core.Point;
//import org.opencv.core.Rect;
//import org.opencv.core.Scalar;
//import org.opencv.core.Size;
//import org.opencv.imgproc.Imgproc;
//import org.openftc.easyopencv.OpenCvPipeline;
//
//import java.util.ArrayList;
//import java.util.List;
//
//public class PixelGroupDetectionPipeline extends OpenCvPipeline {
//    enum Color {
//        WHITE(new Scalar(0, 0, 178.5), new Scalar(255, 26.9, 255), true),
//        YELLOW(new Scalar(18.4, 111.9, 168.6), new Scalar(36.8, 225.0, 255.0), true),
//        PURPLE(new Scalar(121.8, 0, 123.3), new Scalar(137.4, 109.1, 123.3), true),
//        GREEN(new Scalar(38.3, 66.6, 83.6), new Scalar(55.3, 174.3, 255.0), true);
//        public final Scalar UPPER;
//        public final Scalar LOWER;
//        public final boolean display;
//        Color(Scalar upper, Scalar lower, boolean display) {
//            this.UPPER = upper; this.LOWER = lower; this.display = display;
//        }
//    }
//    private static final Color[] colors = new Color[]{Color.WHITE, Color.YELLOW, Color.PURPLE, Color.GREEN};
//
//    private Pixel[] _pixels = new Pixel[2];
//
//    public static class Pixel {
//        public MatOfPoint contour;
//        public Color color;
//        public Pixel(MatOfPoint contour, Color color) {
//            this.contour = contour;
//            this.color = color;
//        }
//    }
//
//    private int impWidth, impHeight;
//
//    @Override
//    public Mat processFrame(Mat input) {
//        impWidth = input.width();
//        impHeight = input.height();
//        Mat output = input.clone();
//        Mat hsv = new Mat();
//        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
//
//        Mat kernel = Imgproc.getStructuringElement(Imgproc.CV_SHAPE_RECT, new Size(25, 25));
//
//        ArrayList<Pixel> pixels = new ArrayList<>();
//        for (Color c : colors) {
//            List<MatOfPoint> contours = new ArrayList<>();
//            Mat inRange = new Mat();
//            Mat denoisedInRange = new Mat();
//            Core.inRange(hsv, c.LOWER, c.UPPER, inRange);
//
//            Imgproc.morphologyEx(inRange, denoisedInRange, Imgproc.MORPH_DILATE, kernel);
//            Imgproc.findContours(denoisedInRange, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
//
//            if (c == Color.PURPLE)
//            {
//                Imgproc.cvtColor(denoisedInRange, output, Imgproc.COLOR_GRAY2RGB);
//            }
//            for (int i = 0; i < contours.size(); i++) {
//                MatOfPoint cont = contours.get(i);
//                MatOfPoint2f c2f = new MatOfPoint2f(cont.toArray());
//                double peri = Imgproc.arcLength(c2f, true);
//                MatOfPoint2f approx = new MatOfPoint2f();
//                Imgproc.approxPolyDP(c2f, approx, 0.05 * peri, true);
//
//                Point[] points = approx.toArray();
//                if (points.length > 0 && Imgproc.contourArea(cont) > input.width() / 10.0) {
//                    MatOfPoint simpleContour = new MatOfPoint(points);
////                if (Imgproc.contourArea(cont) > input.width() / 10.0 && c.display) {
//                    if (Imgproc.isContourConvex(simpleContour)) pixels.add(new Pixel(simpleContour, c));
////                        Imgproc.drawContours(input, contours, i, new Scalar(255, 255, 0), 2);
////                        Imgproc.putText(input, c.name(), new Point(contours.get(i).get(0, 0)), Imgproc.FONT_HERSHEY_PLAIN, 2, new Scalar(0, 255, 0), 2);
//                }
//            }
//
//        }
//        int p_idx=0;
//        while (p_idx<pixels.size()){
//            Pixel p = pixels.get(p_idx);
//            Rect p_rect = Imgproc.boundingRect(p.contour);
//            //top left of p
//            int x1 = p_rect.x;
//            int y1 = p_rect.y;
//            //bottom right of p
//            int x2 = p_rect.x+p_rect.width;
//            int y2 = p_rect.y+p_rect.height;
//
//            //area
//            double p_area = Imgproc.contourArea(p.contour);
//            int p2_idx=0;
//            while (p2_idx<pixels.size()){
////            for (int p2_idx=0; p2_idx<pixels.size(); p2_idx++){
//                Pixel p2 = pixels.get(p2_idx);
//                if (p != p2) {
//                    Rect p2_rect = Imgproc.boundingRect(p2.contour);
//                    //top left of p2
//                    int x3 = p2_rect.x;
////                    int x=x3;
//                    int y3 = p2_rect.y;
//                    //bottom right of p2
//                    int x4 = p2_rect.x+p2_rect.width;
//                    int y4 = p2_rect.y+p2_rect.height;
//
//                    // find intersection:
//                    int xL = Math.max(x1, x3);
//                    int xR = Math.min(x2, x4);
//                    if (xR > xL){
//                        int width = xR - xL;
//                        int yT = Math.max(y1, y3);
//                        int yB = Math.min(y2, y4);
//                        if (yB > yT){
//                            int height = yB - yT;
////                            Imgproc.rectangle(input, new Rect(xL, yT, xR-xL, yB-yT), new Scalar(255,0,0), 5);
//                            double p2_area = Imgproc.contourArea(p2.contour);
//                            try {
//                                if (p_area > p2_area) {
//                                    pixels.remove(pixels.get(p2_idx));
//                                } else {
//                                    pixels.remove(pixels.get(p_idx));
//                                }
//                            } catch (Exception e) {
//                                // adding this catch bc an out of range error (idk why though)
//                            }
//
////                            pixels.remove(p_idx);
////                            Imgproc.rectangle(input, new Point(xL, yB), new Point(xR-xL, yB-yT));
//                        }
////                            return null;
////                        else
////                            return new Rect(xL, yB, xR-xL, yB-yT);
//                    }
//
//
//                }
//                p2_idx++;
//            }
//            p_idx++;
//        }
//        for (Pixel p : pixels){
//            List<MatOfPoint> contours = new ArrayList<>();
//            contours.add(p.contour);
//            Imgproc.drawContours(input, contours, 0, new Scalar(255, 255, 0), 2);
//            putText(input, p.color.name(), new Point(contours.get(0).get(0, 0)), Imgproc.FONT_HERSHEY_PLAIN, 2, new Scalar(0, 255, 0), 2);
//        }
//        double maxArea1 = Double.MIN_VALUE;
//        Pixel maxPixel1 = null;
//        for (Pixel p : pixels) {
//            double a = Imgproc.contourArea(p.contour);
//            if (a > maxArea1) {
//                maxPixel1 = p;
//                maxArea1 = a;
//            }
//        }
//        pixels.remove(maxPixel1);
//        double maxArea2 = Double.MIN_VALUE;
//        Pixel maxPixel2 = null;
//        for (Pixel p : pixels) {
//            double a = Imgproc.contourArea(p.contour);
//            if (a > maxArea2) {
//                maxPixel2 = p;
//                maxArea2 = a;
//            }
//        }
//        _pixels = new Pixel[]{maxPixel1, maxPixel2};
//        Point center = getPixelsCenter();
//        Imgproc.circle(input, new Point(center.x + impWidth / 2.0, center.y + impHeight / 2.0), 1, new Scalar(0, 0, 255), 6);
//        return input;
//    }
//
//    public Pixel[] getPixels() {
//        return _pixels;
//    }
//
//    public Point getPixelsCenter() {
//        Rect[] pxBoundingRects = new Rect[]{_pixels[0] != null ? Imgproc.boundingRect(_pixels[0].contour) : null, _pixels[1] != null ? Imgproc.boundingRect(_pixels[1].contour) : null};
//        if (pxBoundingRects[0] == null) {
//            return new Point(0, 0);
//        }
//        if (pxBoundingRects[1] == null) {
//            return new Point(pxBoundingRects[0].x + pxBoundingRects[0].width / 2.0, pxBoundingRects[0].y + pxBoundingRects[0].height / 2.0);
//        }
//        return new Point(
//                (pxBoundingRects[0].x + pxBoundingRects[0].width / 2.0 + pxBoundingRects[1].x + pxBoundingRects[1].width / 2.0) / 2 - (impWidth / 2.0),
//                (pxBoundingRects[0].y + pxBoundingRects[0].height / 2.0 + pxBoundingRects[1].y + pxBoundingRects[1].height / 2.0) / 2 - (impHeight / 2.0)
//        );
//    }
//}
