package org.firstinspires.ftc.teamcode.cvpipelines;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.openftc.easyopencv.OpenCvPipeline;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.CvType;

import java.util.ArrayList;
import java.util.List;

public class CornerDetectionTesterBinary extends OpenCvPipeline {
    enum PixelColor {
        WHITE(new Scalar(0, 0, 178.5), new Scalar(255, 26.9, 255), true),
        YELLOW(new Scalar(19, 50, 0), new Scalar(80, 225.0, 255.0), true),
        PURPLE(new Scalar(124.7, 36.8, 58.1), new Scalar(151.6, 117.6, 242.3), true),
        GREEN(new Scalar(38.3, 66.6, 83.6), new Scalar(55.3, 174.3, 255.0), true);
        public final Scalar UPPER;
        public final Scalar LOWER;
        public final boolean display;
        PixelColor(Scalar lower, Scalar upper, boolean display) {
            this.UPPER = upper; this.LOWER = lower; this.display = display;
        }
    }

    public static PixelColor[] colors = new PixelColor[]{PixelColor.WHITE, PixelColor.YELLOW, PixelColor.PURPLE, PixelColor.GREEN};

    public Mat frame;
    public int getCameraWidth() {
        return frame.width();
    }

    public static Telemetry telemetry;

    private int centX = 0;
    private int centY = 0;

    public Point getPixelsCenter() {
        return new Point(centX, centY);
    }

    public static Scalar testVariable = new Scalar(0, 1, 2);
    public static Scalar testVariable2 = new Scalar(0, 1, 2);

    public static Scalar lowerYellow = new Scalar(12.8, 86.4, 137.8); // hsv
    public static Scalar upperYellow = new Scalar(22.7, 255, 255.0); // hsv
    public static int harrisKSize = 3;

//    @Override
//    public void init(int width, int height, CameraCalibration calibration) {
//
//    }

    public CornerDetectionTesterBinary(Telemetry telemetry){
        this.telemetry = telemetry;

    }

    @Override
    public Mat processFrame(Mat input) {
        frame = input;
        Mat output = new Mat();
        Mat hsv = new Mat(); // convert to hsv
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        Mat inRange = new Mat();
//        Core.inRange(hsv, PixelColor.YELLOW.LOWER, PixelColor.YELLOW.UPPER, inRange);
        Core.inRange(hsv, lowerYellow, upperYellow, inRange);
        // inRange is the Binary mask

        // Morphology
        Mat kernel = Imgproc.getStructuringElement(Imgproc.CV_SHAPE_RECT, new Size(20, 20));
        Mat kernel2 = Imgproc.getStructuringElement(Imgproc.CV_SHAPE_RECT, new Size(5, 5));
        Imgproc.dilate(inRange, inRange, kernel);
        Imgproc.erode(inRange, inRange, kernel);

        Mat gray = new Mat();
        Core.multiply(inRange, new Scalar(255), gray);
        // gray is perfectly black and white
//        inRange.copyTo(gray);
//        Imgproc.cvtColor(inRange, gray, Imgproc.COLOR_RGB2GRAY);


        // Convert to float32 for cornerHarris
//        Mat grayFloat = new Mat();
//        gray.convertTo(grayFloat, CvType.CV_32F);

        Mat bi = new Mat();
        Imgproc.bilateralFilter(gray, bi, 5, 75, 75);

        // Apply Harris Corner detection
        Mat corners = new Mat();
        Imgproc.cornerHarris(bi, corners, 2, harrisKSize, 0.02);
        Mat simpleKernel = Mat.ones(new Size(3, 3), CvType.CV_8U);
        Mat dilatedCorners = new Mat();
        Imgproc.dilate(corners, dilatedCorners, simpleKernel);

        // Create a mask to identify corners
        Mat mask = Mat.zeros(corners.size(), CvType.CV_8UC1);
        Core.compare(dilatedCorners, new Scalar(0.01 * Core.minMaxLoc(dilatedCorners).maxVal), mask, Core.CMP_GT);

        // Find the coordinates of the corners
        List<Point> cornerPoints = new ArrayList<>();
        for (int y = 0; y < mask.rows(); y++) {
            for (int x = 0; x < mask.cols(); x++) {
                if (mask.get(y, x)[0] > 0) {
                    cornerPoints.add(new Point(x, y));
                }
            }
        }

        // Apply a distance threshold to filter close corners
        double thresh = 20;
        List<Point> filteredCorners = filterCornersByDistance(cornerPoints, thresh);

        // Draw the corners on the input frame
        for (Point corner : filteredCorners) {
            Imgproc.circle(input, corner, 5, new Scalar(0, 0, 255), -1);
        }
//
//        Imgproc.cvtColor(inRange, inRange, Imgproc.COLOR_GRAY2RGBA);
////        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGBA2BGR);
//        int thresh = 20;
//        for( int j = 0; j < dilatedCorners.rows() ; j++){
//            for( int i = 0; i < dilatedCorners.cols(); i++){
//                if ((int) dilatedCorners.get(j,i)[0] > thresh){
//                    Imgproc.circle(inRange, new Point(i,j), 5 , new Scalar(100, 0, 155), 2 ,8 , 0);
//                }
//            }
//        }

        telemetry.addData("ee", inRange.type());
        telemetry.addData("input", input.type());
        telemetry.addData("corners", corners.type());
        telemetry.update();
//        Core.bitwise_and(input, inRange, output);
//        Core.bitwise_or(output, corners, output);

        return input;


//        Mat kernel = Imgproc.getStructuringElement(Imgproc.CV_SHAPE_RECT, new Size(15, 15));
//        for (PixelColor c : colors) {
//            if (c != PixelColor.YELLOW) {
//                continue;
//            }
//            Core.inRange(hsv, c.LOWER, c.UPPER, inRange);
//        }
//        return inRange;
    }

    private List<Point> filterCornersByDistance(List<Point> corners, double thresh) {
        List<Point> filteredCorners = new ArrayList<>(corners);

        for (int i = 0; i < filteredCorners.size(); i++) {
            Point pt1 = filteredCorners.get(i);
            for (int j = i + 1; j < filteredCorners.size(); j++) {
                Point pt2 = filteredCorners.get(j);
                if (distance(pt1, pt2) < thresh) {
                    filteredCorners.remove(j);
                    j--; // Adjust index after removal
                }
            }
        }

        return filteredCorners;
    }

    private double distance(Point pt1, Point pt2) {
        return Math.sqrt(Math.pow(pt2.x - pt1.x, 2) + Math.pow(pt2.y - pt1.y, 2));
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
//        Paint p = new Paint();
//        p.setColor(Color.BLUE);
//        p.setStrokeWidth(4);
//        canvas.drawCircle((float) getPixelsCenter().x, (float) getPixelsCenter().y, 0, p);
//        canvas.drawCircle((float) getPixelsCenter().x, (float) getPixelsCenter().y, 6, p);
    }
}