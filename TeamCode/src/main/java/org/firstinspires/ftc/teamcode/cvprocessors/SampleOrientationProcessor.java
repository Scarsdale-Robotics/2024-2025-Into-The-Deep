package org.firstinspires.ftc.teamcode.cvprocessors;

import android.graphics.Canvas;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

@Config
public class SampleOrientationProcessor implements VisionProcessor {
    public enum SampleColor {
        YELLOW(),
        BLUE(),
        RED()
    }

    private Mat frame;

    public static double cameraHeight = 8.409; // inches (-1.5 because of sample height)


    public static Scalar lowerYellow = new Scalar(15.0, 150.0, 160.0); // hsv
    public static Scalar upperYellow = new Scalar(30.0, 255.0, 255.0); // hsv
    public static Scalar lowerBlue = new Scalar(90.0, 100.0, 120.0); // hsv
    public static Scalar upperBlue = new Scalar(140.0, 255.0, 255.0); // hsv
    public static Scalar lowerRedH = new Scalar(10.0, 0.0, 0.0); // hsv
    public static Scalar upperRedH = new Scalar(160.0, 255.0, 255.0); // hsv
    public static Scalar lowerRedSV = new Scalar(0.0, 160.0, 160.0); // hsv
    public static Scalar upperRedSV = new Scalar(255.0, 255.0, 255.0); // hsv

    public static double AREA_PER_SAMPLE = 6000d;
    public static double SAMPLE_LONG_LENGTH = 120;
    public static double SAMPLE_SHORT_LENGTH = 50;

    private volatile double averageBrightness = -1;
    private volatile ArrayList<double[]> realPositions = new ArrayList<>();
    private volatile ArrayList<Double> sampleAngles = new ArrayList<>();

    public static volatile SampleColor colorType = SampleColor.YELLOW;


    public static double horizontalBiasTune = 1.2;


    public SampleOrientationProcessor() {
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
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
        Imgproc.morphologyEx(inRange, inRange, Imgproc.MORPH_OPEN, kernel);

        // Find all contours
        List<MatOfPoint> unfilteredContours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(inRange, unfilteredContours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        // Filter contours by size
        int minArea = 5500;
        List<MatOfPoint> largeContours = new ArrayList<>();
        ArrayList<Integer> largeContoursSampleCount = new ArrayList<>();
        ArrayList<RotatedRect> rotatedRects = new ArrayList<>();
        for (MatOfPoint contour : unfilteredContours) {
            // check area
            double area = Imgproc.contourArea(contour);
            if (area < minArea) continue;

            if (area < AREA_PER_SAMPLE) {
                // check if touching edge
                Rect boundingRect = Imgproc.boundingRect(contour);
                double[][] corners = {
                        {boundingRect.x, boundingRect.y},
                        {boundingRect.x + boundingRect.width, boundingRect.y + boundingRect.height}
                };
                double frameMargin = 1;
                boolean validRect = true;
                for (double[] corner : corners) {
                    double cornerX = corner[0];
                    double cornerY = corner[1];
                    if (cornerX <= frameMargin || cornerX >= frame.width() - frameMargin || cornerY <= frameMargin || cornerY >= frame.height() - frameMargin) {
                        validRect = false;
                        break;
                    }
                }
                if (!validRect) continue; // so it's a valid contour
            }

            // detect if it's a large contour
            int sampleCount = (int)Math.round(area/AREA_PER_SAMPLE);
            if (sampleCount<=1) {
                rotatedRects.add(Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray())));
            } else {
                largeContours.add(contour);
                largeContoursSampleCount.add(sampleCount);
            }
        }

        // Prepare to process to extract each rectangle
        double resolutionDivisor = 8;
        Mat workingImg = new Mat();
        int newWidth = frame.cols() / (int)resolutionDivisor;
        int newHeight = frame.rows() / (int)resolutionDivisor;
        Size newSize = new Size(newWidth, newHeight);
        Imgproc.resize(inRange, workingImg, newSize);

        // Approximate large contours with polygons
        for (int contourIdx = 0; contourIdx < largeContours.size(); contourIdx++) {
            MatOfPoint contour = largeContours.get(contourIdx);

            // Convert contour to MatOfPoint2f for approximation
            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
            MatOfPoint2f approxCurve = new MatOfPoint2f();

            // Epsilon controls the level of approximation
            double epsilon = 5;
            Imgproc.approxPolyDP(contour2f, approxCurve, epsilon, true);

            // Extract rectangle edges (150 or 350 pixels)
            List<Point[]> candidateEdges = new ArrayList<>();
            List<Boolean> candidateEdgeLengths = new ArrayList<>(); // true = short side
            Point[] polyPoints = approxCurve.toArray();
            for (int i = 0; i < polyPoints.length; i++) {
                Point p1 = polyPoints[i];
                Point p2 = polyPoints[(i + 1) % polyPoints.length]; // Wrap around

                double length = Math.hypot(p2.x - p1.x, p2.y - p1.y);
                if (Math.abs(length - SAMPLE_SHORT_LENGTH) < 10) {
                    candidateEdges.add(new Point[]{p1, p2});
                    candidateEdgeLengths.add(true);
                }
                else if (Math.abs(length - SAMPLE_LONG_LENGTH) < 10) {
                    candidateEdges.add(new Point[]{p1, p2});
                    candidateEdgeLengths.add(false);
                }
                Imgproc.line(frame, p1, p2, new Scalar(0, 255, 255), 1);
            }

            for (int r = 0; r < largeContoursSampleCount.get(contourIdx); r++) {
                // initialize parameters for detecting RotatedRects
                int greatestAreaIndex = -1;
                double greatestIntersectionArea = -1;
                RotatedRect greatestAreaRotatedRect = null;
                for (int i = 0; i < candidateEdges.size(); i++) {
                    Point[] edge = candidateEdges.get(i);
                    Point p1 = edge[0];
                    Point p2 = edge[1];
                    double dx = p2.x - p1.x;
                    double dy = p2.y - p1.y;

                    Point mp = new Point(
                            p1.x + dx / 2,
                            p1.y + dy / 2
                    );

                    // Find perpendicular direction
                    double lineTheta = Math.atan2(dy, dx);
                    double theta1 = lineTheta + Math.PI / 2;
                    double theta2 = lineTheta - Math.PI / 2;
                    RotatedRect rect1 = createRotatedRect(mp, theta1, candidateEdgeLengths.get(i));
                    RotatedRect rect2 = createRotatedRect(mp, theta2, candidateEdgeLengths.get(i));

                    // Check if rects are touching edge
                    double frameMargin = 1;
                    Point[] corners1 = new Point[4];
                    Point[] corners2 = new Point[4];
                    rect1.points(corners1);
                    rect2.points(corners2);
                    boolean validRect1 = true;
                    boolean validRect2 = true;
                    for (int cornerIdx = 0; cornerIdx < 4; cornerIdx++) {
                        // check rect 1
                        Point corner1 = corners1[cornerIdx];
                        double cornerX1 = corner1.x;
                        double cornerY1 = corner1.y;
                        if (validRect1 && (cornerX1 <= frameMargin || cornerX1 >= frame.width() - frameMargin || cornerY1 <= frameMargin || cornerY1 >= frame.height() - frameMargin)) {
                            validRect1 = false;
                        }
                        // check rect 2
                        Point corner2 = corners2[cornerIdx];
                        double cornerX2 = corner2.x;
                        double cornerY2 = corner2.y;
                        if (validRect2 && (cornerX2 <= frameMargin || cornerX2 >= frame.width() - frameMargin || cornerY2 <= frameMargin || cornerY2 >= frame.height() - frameMargin)) {
                            validRect2 = false;
                        }
                    }

                    // Compute pixel count inside each mask
                    double intersectionArea1 = validRect1 ? getIntersectionArea(workingImg, rect1, resolutionDivisor) : -1;
                    double intersectionArea2 = validRect2 ? getIntersectionArea(workingImg, rect2, resolutionDivisor) : -1;

                    // Compare to max intersection area
                    if (validRect1 && intersectionArea1 > greatestIntersectionArea) {
                        greatestAreaIndex = i;
                        greatestIntersectionArea = intersectionArea1;
                        greatestAreaRotatedRect = rect1;
                    }
                    if (validRect2 && intersectionArea2 > greatestIntersectionArea) {
                        greatestAreaIndex = i;
                        greatestIntersectionArea = intersectionArea2;
                        greatestAreaRotatedRect = rect2;
                    }
                }

                if (greatestAreaRotatedRect==null) continue;
                if (greatestIntersectionArea<minArea/(resolutionDivisor*resolutionDivisor)) continue;

                // Erase greatest intersecting RotatedRect from working image
                Point[] originalVertices = new Point[4];
                Point[] scaledVertices = new Point[4];
                greatestAreaRotatedRect.points(originalVertices);
                for (int i = 0; i < 4; i++) {
                    Point vertex = originalVertices[i];
                    scaledVertices[i] = new Point(
                            vertex.x/resolutionDivisor,
                            vertex.y/resolutionDivisor
                    );
                }
                MatOfPoint matOfPoint = new MatOfPoint(scaledVertices);
                Imgproc.fillPoly(workingImg, Collections.singletonList(matOfPoint), new Scalar(0, 0, 0));

                // Remove edge from list
                candidateEdges.remove(greatestAreaIndex);
                candidateEdgeLengths.remove(greatestAreaIndex);

                // Save rotated rect
                rotatedRects.add(greatestAreaRotatedRect);

            }
        }


        // Draw filtered rects as green
        ArrayList<double[]> tempRealPositions = getOffsets(rotatedRects, scalingFactor);
        realPositions = tempRealPositions;
        for (RotatedRect rotatedRect : rotatedRects) {
            Point[] vertices = new Point[4];
            rotatedRect.points(vertices);
            for (int j = 0; j < 4; j++) {
                Imgproc.line(input, vertices[j], vertices[(j + 1) % 4], new Scalar(0, 255, 0), 2);
            }
            Imgproc.circle(input, rotatedRect.center, 5, new Scalar(255, 255, 0));
        }


        // Update sample angles list
        ArrayList<Double> tempSampleAngles = new ArrayList<>();
        for (RotatedRect rect : rotatedRects) {
            double procAngle = rect.angle;
            if (rect.size.width > rect.size.height)
                procAngle *= -1;
            else
                procAngle = 90-procAngle;
            while (procAngle>0) procAngle -= 180;
            procAngle -= 90;
            if (procAngle < -90) procAngle += 180;
            tempSampleAngles.add(Math.toRadians(procAngle));
        }
        sampleAngles = tempSampleAngles;


        return input;
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

    public static RotatedRect createRotatedRect(Point p, double theta, boolean shortSide) {
        double halfSideLength;
        if (shortSide) halfSideLength = SAMPLE_LONG_LENGTH/2;
        else halfSideLength = SAMPLE_SHORT_LENGTH/2;
        // Compute center of rectangle
        double centerX = p.x + halfSideLength * Math.cos(theta);
        double centerY = p.y + halfSideLength * Math.sin(theta);
        Point center = new Point(centerX, centerY);

        // Create the RotatedRect (width=300, height=100)
        if (shortSide) {
            return new RotatedRect(center, new Size(SAMPLE_LONG_LENGTH, SAMPLE_SHORT_LENGTH), Math.toDegrees(theta));
        } else {
            return new RotatedRect(center, new Size(SAMPLE_SHORT_LENGTH, SAMPLE_LONG_LENGTH), Math.toDegrees(theta));
        }
    }

    public static double getIntersectionArea(Mat binaryImage, RotatedRect rotatedRect, double resolutionDivisor) {
        // Create a mask for the rotated rectangle
        Mat mask = new Mat(binaryImage.size(), CvType.CV_8UC1, new Scalar(0)); // Black mask
        Point[] originalVertices = new Point[4];
        Point[] scaledVertices = new Point[4];
        rotatedRect.points(originalVertices);
        for (int i = 0; i < 4; i++) {
            Point vertex = originalVertices[i];
            scaledVertices[i] = new Point(
                    vertex.x/resolutionDivisor,
                    vertex.y/resolutionDivisor
            );
        }
        MatOfPoint matOfPoint = new MatOfPoint(scaledVertices);

        // Fill the mask inside the rotated rectangle with white
        Imgproc.fillPoly(mask, java.util.Collections.singletonList(matOfPoint), new Scalar(255));

        // Perform bitwise AND to find the intersection
        Mat intersection = new Mat();
        Core.bitwise_and(binaryImage, mask, intersection);

        // Calculate the area of the intersection (number of non-zero pixels)
        return Core.countNonZero(intersection);
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

        double horizontalBias = horizontalBiasTune;//1.142857;

        double scaled320 = 320/scalingFactor;
        double scaled240 = 240/scalingFactor;
        for (RotatedRect i : input) {
            // real center is (320, 480), positive direction is right and down
//            output.add(new Point(i.center.x - 320, -(i.center.y - 240)));
            double horizontalCoordinate = (i.center.x - scaled320) / scaled320 * canvasHorizontal * horizontalBias;
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

}
