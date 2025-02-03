package org.firstinspires.ftc.teamcode.cvprocessors;

import android.graphics.Canvas;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

@Config
public class SampleOrientationProcessor implements VisionProcessor {
    public enum SampleColor {
        YELLOW(),
        BLUE(),
        RED();
    }

    private Mat frame;

    private Telemetry telemetry;

    public static double cameraHeight = 11.291339 - 1.5; // inches (-1.5 because of sample height)

    public static Scalar lowerYellow = new Scalar(19.0, 102.0, 130.1); // hsv
    public static Scalar upperYellow = new Scalar(30.0, 255.0, 255.0); // hsv
    public static Scalar lowerBlue = new Scalar(90.0, 90.0, 90.0); // hsv
    public static Scalar upperBlue = new Scalar(120.0, 255.0, 255.0); // hsv
    public static Scalar lowerRedH = new Scalar(10.0, 0.0, 0.0); // hsv
    public static Scalar upperRedH = new Scalar(170.0, 255.0, 255.0); // hsv
    public static Scalar lowerRedSV = new Scalar(0.0, 130.0, 100.0); // hsv
    public static Scalar upperRedSV = new Scalar(255.0, 255.0, 255.0); // hsv

    private volatile boolean sampleDetected = false;
    private volatile double sampleAngle = 0;
    private volatile double averageBrightness = 0;
    private volatile ArrayList<double[]> realPositions = new ArrayList<>();
    private volatile ArrayList<Double> sampleAngles = new ArrayList<>();

    public static volatile SampleColor colorType = SampleColor.YELLOW;

    public SampleOrientationProcessor(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat input, long captureTimeNanos) {
        frame = input.clone();
        double scalingFactor = (double) 640 /frame.width();


        // Getting representative brightness of image
        Mat gray = new Mat(); // convert to hsv
        Imgproc.cvtColor(input, gray, Imgproc.COLOR_RGB2GRAY);
        MatOfDouble muMat = new MatOfDouble();
        MatOfDouble sigmaMat = new MatOfDouble();
        Core.meanStdDev(gray, muMat, sigmaMat);
        double mu = muMat.get(0,0)[0];
        double sigma = sigmaMat.get(0,0)[0];
        double k = 1;

        Scalar lowerBound = new Scalar(mu-k*sigma);
        Scalar upperBound = new Scalar(mu+k*sigma);
        Mat mask = new Mat();
        Core.inRange(gray, lowerBound, upperBound, mask);
        Scalar maskedMean = Core.mean(gray, mask);
        averageBrightness = maskedMean.val[0];
//        telemetry.addData("averageBrightness", averageBrightness);
        double targetAverageInRange = 120;
        frame.convertTo(frame, -1, targetAverageInRange/ averageBrightness, 0);


        // Color threshold
        Mat hsv = new Mat(); // convert to hsv
        Imgproc.cvtColor(frame, hsv, Imgproc.COLOR_RGB2HSV);
        Mat inRange = new Mat();
        if (colorType.equals(SampleColor.BLUE)) {
            Core.inRange(hsv, lowerBlue, upperBlue, inRange);
        } else if (colorType.equals(SampleColor.RED)) {
            Mat inHRange = new Mat();
            Mat inSVRange = new Mat();
            Core.inRange(hsv, lowerRedH, upperRedH, inHRange);
            Core.bitwise_not(inHRange, inHRange);
            Core.inRange(hsv, lowerRedSV, upperRedSV, inSVRange);
            Core.bitwise_and(inHRange, inSVRange, inRange);
        } else {
            Core.inRange(hsv, lowerYellow, upperYellow, inRange);
        }


        // Morphology
        Mat kernel = Imgproc.getStructuringElement(Imgproc.CV_SHAPE_RECT, new Size(3/Math.sqrt(scalingFactor), 3/scalingFactor));
        Imgproc.erode(inRange, inRange, kernel);


        // Find all contours
        List<MatOfPoint> unfilteredContours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(inRange, unfilteredContours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);


        // Filter contours by size and get rotated rects
        int minArea = (int)(5000/(scalingFactor*scalingFactor));
        ArrayList<RotatedRect> rotatedRects = new ArrayList<>();
        List<MatOfPoint> filteredContours = new ArrayList<>();
        for (MatOfPoint contour : unfilteredContours) {
            RotatedRect minAreaRect = Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray()));
            double area = minAreaRect.size.area();
            if (area > minArea) {
                filteredContours.add(contour);
                rotatedRects.add(minAreaRect);
            }
        }
        Imgproc.drawContours(frame, filteredContours, -1, new Scalar(0, 255, 0), 2);


        // Get overlapping rotated rect groups
        double overlapThreshold = 0.2; // % of smaller box covered
        Set<Integer> toSkip = new HashSet<>();
        ArrayList<ArrayList<Double[]>> overlapGroups = new ArrayList<>();
        for (int i = 0; i < rotatedRects.size(); i++) {
            if (toSkip.contains(i)) continue;
            toSkip.add(i);
            ArrayList<Double[]> overlapGroup = new ArrayList<>();
            double iArea = rotatedRects.get(i).size.area();
            overlapGroup.add(new Double[]{(double)i, iArea});
            for (int j = i+1; j < rotatedRects.size(); j++) {
                if (toSkip.contains(j)) continue;
                double jArea = rotatedRects.get(j).size.area();
                for (Double[] rect : overlapGroup) {
                    double overlapArea = getIntersectionArea(rotatedRects.get(rect[0].intValue()), rotatedRects.get(j));
                    if (overlapArea / Math.min(rect[1], jArea) >= overlapThreshold) {
                        overlapGroup.add(new Double[]{(double)j, jArea});
                        toSkip.add(j);
                        break;
                    }
                }
            }
            overlapGroups.add(overlapGroup);
        }


        // Filter out overlapping rects
        ArrayList<RotatedRect> filteredRects = new ArrayList<>();
        for (ArrayList<Double[]> overlapGroup : overlapGroups) {
            int maxIndex = overlapGroup.get(0)[0].intValue();
            double maxArea = overlapGroup.get(0)[1];
            for (Double[] rect : overlapGroup) {
                if (rect[1] > maxArea) {
                    maxArea = rect[1];
                    maxIndex = rect[0].intValue();
                }
            }
            filteredRects.add(rotatedRects.get(maxIndex));
        }
//        telemetry.addData("filteredRects.size()", filteredRects.size());


        // Draw filtered rects as green
        ArrayList<double[]> tempRealPositions = getOffsets(filteredRects, scalingFactor);
        realPositions = tempRealPositions;
        for (RotatedRect rotatedRect : filteredRects) {
            Point[] vertices = new Point[4];
            rotatedRect.points(vertices);
            for (int j = 0; j < 4; j++) {
                Imgproc.line(frame, vertices[j], vertices[(j + 1) % 4], new Scalar(0, 255, 0), 2);
            }
            Imgproc.circle(frame, rotatedRect.center, 5, new Scalar(255, 255, 0));
        }


        // Update sample angles list
        ArrayList<Double> tempSampleAngles = new ArrayList<>();
        for (RotatedRect rect : filteredRects) {
            double procAngle = rect.angle;
            if (rect.size.width > rect.size.height)
                procAngle *= -1;
            else
                procAngle = 90-procAngle;
            procAngle -= 90;
            if (procAngle < -90) procAngle += 180;
            tempSampleAngles.add(Math.toRadians(procAngle));
        }
        sampleAngles = tempSampleAngles;


        // telemetry
        if (!filteredRects.isEmpty()) {
//            telemetry.addData("width ", filteredRects.get(0).size.width);
//            telemetry.addData("height ", filteredRects.get(0).size.height);
//            telemetry.addData("angle ", filteredRects.get(0).angle);
//            telemetry.addData("center ", filteredRects.get(0).center);
            double procAngle = filteredRects.get(0).angle;
            if (filteredRects.get(0).size.width > filteredRects.get(0).size.height)
                procAngle *= -1;
            else
                procAngle = 90-procAngle;
//            telemetry.addData("procAngle ", procAngle);
            sampleAngle = Math.toRadians(procAngle);
            sampleDetected = true;
        } else {
            sampleDetected = false;
        }
//        telemetry.addData("sampleAngle", sampleAngle);



//        telemetry.update();
        return frame;
    }

    public synchronized boolean getSampleDetected() {
        return sampleDetected;
    }

    public synchronized double getFirstSampleAngle() {
        return sampleAngle;
    }

    public synchronized double getAverageBrightness() {
        return averageBrightness;
    }

    public synchronized ArrayList<double[]> getRealPositions() {
        return realPositions;
    }

    public synchronized ArrayList<Double> getSampleAngles() {
        return sampleAngles;
    }

    public synchronized void setFilterColor(SampleColor color) {
        colorType = color;
    }

    private double getIntersectionArea(RotatedRect rect1, RotatedRect rect2) {
        // Get vertices of the rectangles
        Point[] vertices1 = new Point[4];
        rect1.points(vertices1);

        Point[] vertices2 = new Point[4];
        rect2.points(vertices2);

        // Convert vertices arrays to MatOfPoint2f
        MatOfPoint2f poly1 = new MatOfPoint2f(vertices1);
        MatOfPoint2f poly2 = new MatOfPoint2f(vertices2);

        // Output MatOfPoint2f for the intersection polygon
        MatOfPoint2f intersection = new MatOfPoint2f();

        // Calculate intersection
        return Imgproc.intersectConvexConvex(poly1, poly2, intersection, true);
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    public ArrayList<double[]> getOffsets(ArrayList<RotatedRect> input, double scalingFactor) {
        // Note: This method only works when the camera is directly above the samples, looking straight down

        ArrayList<double[]> output = new ArrayList<>();

        // TODO: Make height not hardcoded, instead base it off of robot position
        double height = cameraHeight; // in inches
        double canvasVertical = 1.1 * height*3.0/8.0; // inches
        double canvasHorizontal = 1.1 * height / 2;

        double scaled320 = 320/scalingFactor;
        double scaled240 = 240/scalingFactor;
        for (RotatedRect i : input) {
            // real center is (320, 480), positive direction is right and down
//            output.add(new Point(i.center.x - 320, -(i.center.y - 240)));
            double horizontalCoordinate = (i.center.x - scaled320) / scaled320 * canvasHorizontal;
            double verticalCoordinate = -(i.center.y - scaled240) / scaled240 * canvasVertical;
            output.add(new double[]{verticalCoordinate, -horizontalCoordinate});


            // 4 in height = 1.5 width vertical (half width, not full)
            // 6 : 2.25
            // 2 : 0.75
            // 8 : 3
            // horizontal: 8 / 4
        }

        return output;
    }


    public ArrayList<double[]> getOffsetsWithHeights(ArrayList<RotatedRect> input, double scalingFactor) {
        // Note: This method only works when the camera is directly above the samples, looking straight down

        ArrayList<double[]> output = new ArrayList<>();

        double scaled320 = 320/scalingFactor;
        double scaled240 = 240/scalingFactor;
        for (RotatedRect i : input) {
            // real center is (320, 480), positive direction is right and down
//            output.add(new Point(i.center.x - 320, -(i.center.y - 240)));


            // get real sample coords
            double area = i.size.area()*scalingFactor*scalingFactor; //*2.091295825;
            double sampleHeight = 1435.0/Math.sqrt(area)-0.308; // calculate height of camer based on area of sample

            double canvasVertical = 1.1 * sampleHeight*3.0/8.0; // inches
            double canvasHorizontal = 1.1 * sampleHeight / 2;

            output.add(new double[]{(i.center.x - scaled320) / scaled320 * canvasHorizontal, -(i.center.y - scaled240) / scaled240 * canvasVertical, sampleHeight});


            // 4 in height = 1.5 width vertical (half width, not full)
            // 6 : 2.25
            // 2 : 0.75
            // 8 : 3
            // horizontal: 8 / 4
        }


        return output;
    }
}
