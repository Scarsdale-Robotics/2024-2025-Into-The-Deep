package org.firstinspires.ftc.teamcode.cvpipelines;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

public class RectDrawer_suspicious extends OpenCvPipeline {

    public enum SampleColor {
        YELLOW(),
        BLUE(),
        RED();
    }

    public Mat frame;

    public static Telemetry telemetry;

    public static Scalar lowerYellow = new Scalar(15.0, 100.0, 100.1); // hsv
    public static Scalar upperYellow = new Scalar(30.0, 255.0, 255.0); // hsv
    public static Scalar lowerBlue = new Scalar(90.0, 140.0, 100.0); // hsv
    public static Scalar upperBlue = new Scalar(140.0, 255.0, 255.0); // hsv
    public static Scalar lowerRedH = new Scalar(10.0, 0.0, 0.0); // hsv
    public static Scalar upperRedH = new Scalar(160.0, 255.0, 255.0); // hsv
    public static Scalar lowerRedSV = new Scalar(0.0, 100.0, 100.0); // hsv
    public static Scalar upperRedSV = new Scalar(255.0, 255.0, 255.0); // hsv

    private double sampleAngle = 0;

    public static double AREA_PER_SAMPLE = 6000d;
    public static double SAMPLE_LONG_LENGTH = 120;
    public static double SAMPLE_SHORT_LENGTH = 50;

    public static SampleColor colorType = SampleColor.YELLOW;
//    @Override
//    public void init(int width, int height, CameraCalibration calibration) {
//
//    }

    public RectDrawer_suspicious(Telemetry telemetry){
        this.telemetry = telemetry;
//        this.colorType = colorType;
    }

    @Override
    public Mat processFrame(Mat input) {
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
        double averageBrightness = maskedMean.val[0];
        telemetry.addData("averageBrightness", averageBrightness);
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



        // Draw filtered rects as green and their corresponding data
        ArrayList<Point> real = getOffsets(rotatedRects, scalingFactor);
        for (int i = 0; i < rotatedRects.size(); i++) {
            RotatedRect rotatedRect = rotatedRects.get(i);
            Point[] vertices = new Point[4];
            rotatedRect.points(vertices);
            for (int j = 0; j < 4; j++) {
                Imgproc.line(frame, vertices[j], vertices[(j + 1) % 4], new Scalar(0, 255, 0), 2);
            }
            Point center = rotatedRect.center;



            double procAngle = rotatedRect.angle;
            if (rotatedRects.get(i).size.width > rotatedRects.get(i).size.height)
                procAngle *= -1;
            else
                procAngle = 90-procAngle;
            while (procAngle>0) procAngle -= 180;

            double length = 200/scalingFactor;
            double dX = length*Math.cos(Math.toRadians(procAngle));
            double dY = length*Math.sin(Math.toRadians(procAngle));
            double vecX = center.x+dX;
            double vecY = center.y-dY;
            telemetry.addData("vecX", vecX);
            telemetry.addData("vecY", vecY);
            double scaled640 = 640/scalingFactor;
            double scaled480 = 480/scalingFactor;
            if (vecX < 0) {
                vecY = center.y - (1+vecX/dX)*dY;
                vecX = 0;
            } if (vecY < 0) {
                vecX = center.x + (1+vecY/dY)*dX;
                vecY = 0;
            } if (vecX > scaled640) {
                vecY = center.y - (1-(vecX-scaled640)/dX)*dY;
                vecX = scaled640;
            } if (vecY > scaled480) {
                vecX = center.x + (1+(vecY-scaled480)/dY)*dX;
                vecY = scaled480;
            }
            dX = vecX-center.x;
            dY = vecY-center.y;
            Imgproc.line(frame, center, new Point(vecX,vecY), new Scalar(0, 255, 255), 1);
            Imgproc.line(frame, center, new Point(center.x+dY/3, center.y-dX/3), new Scalar(255, 0, 255), 1);
            Imgproc.line(frame, new Point(center.x+dY/10, center.y-dX/10), new Point(center.x+dY/10+dX/10, center.y-dX/10+dY/10), new Scalar(255, 0, 255), 1);
            Imgproc.line(frame, new Point(center.x+dY/10+dX/10, center.y-dX/10+dY/10), new Point(center.x+dX/10, center.y+dY/10), new Scalar(255, 0, 255), 1);
            Imgproc.line(frame, center, new Point(center.x+length/2,center.y), new Scalar(0, 255, 255), 1);
            // Define arc parameters
            int radius = (int)(20/scalingFactor);
            double endAngle = -procAngle;
            Scalar color = new Scalar(0, 255, 255);
            int thickness = 1;

            // Generate points on the arc
            MatOfPoint points = new MatOfPoint();
            Imgproc.ellipse2Poly(center, new Size(radius, radius), 0, 0, (int) endAngle, 1, points);

            // Draw the arc
            List<MatOfPoint> listThing = new ArrayList<>();
            listThing.add(points);
            Imgproc.polylines(frame, listThing, false, color, thickness);
            Imgproc.putText(frame, (Math.round(10*procAngle)/10d)+" deg", new Point(center.x+30/scalingFactor, center.y-10/scalingFactor+(procAngle<0 ? 30 : 0)/scalingFactor), 0, 0.5/scalingFactor, new Scalar(0, 255, 255));


            // get real sample coords
            double area = rotatedRect.size.area()*scalingFactor*scalingFactor; //*2.091295825;
            double sampleHeight = 1435.0/Math.sqrt(area)-0.308; // calculate height of camer based on area of sample
            telemetry.addData("sampleHeight", sampleHeight);


            // TODO: remove
            real = getOffsets(rotatedRects, sampleHeight, scalingFactor);






            double real_x = real.get(i).x; // in inches
            double real_y = real.get(i).y;
            double scaled320 = 320/scalingFactor;
            double scaled240 = 240/scalingFactor;
            telemetry.addData("real_x", real_x);
            telemetry.addData("real_y", real_y);
            Imgproc.line(frame, center, new Point(scaled320, center.y), new Scalar(255, 255, 0), 1);
            Imgproc.putText(frame, (Math.round(10*real_x)/10d)+(Math.abs(real_x)>1?" in":""), new Point(scaled320+(center.x-scaled320)*0.5-20/scalingFactor, center.y+15/scalingFactor), 0, 0.5/scalingFactor, new Scalar(255, 255, 0));
            Imgproc.line(frame, new Point(scaled320, center.y), new Point(scaled320,scaled240), new Scalar(255, 255, 0), 1);
            Imgproc.putText(frame, (Math.round(10*real_y)/10d)+(Math.abs(real_y)>1?" in":""), new Point(scaled320-5/scalingFactor-10*Double.toString(Math.round(10*real_y)/10d).length()/scalingFactor-(Math.abs(real_y)>1?22:0)/scalingFactor, scaled240+(center.y-scaled240)*0.5+10/scalingFactor), 0, 0.5/scalingFactor, new Scalar(255, 255, 0));




            Imgproc.circle(frame, rotatedRect.center, 1, new Scalar(255, 255, 0), 3);
        }
        Imgproc.circle(frame, new Point(320, 240), 1, new Scalar(255, 255, 0), 3);


        // telemetry
        if (!rotatedRects.isEmpty()) {
            telemetry.addData("width ", rotatedRects.get(0).size.width);
            telemetry.addData("height ", rotatedRects.get(0).size.height);
            telemetry.addData("area ", rotatedRects.get(0).size.area());
            telemetry.addData("angle ", rotatedRects.get(0).angle);
            telemetry.addData("center ", rotatedRects.get(0).center);
//            telemetry.addData("center scaled", new Point((filteredRects.get(0).center.x - 320) / 640 * 3.0/8.0, -(filteredRects.get(0).center.y - 240) / 480));
//            output.add(new Point((i.center.x - 320) / 640 * canvasHorizontal, -(i.center.y - 240) / 240 * canvasVertical));
            double procAngle = rotatedRects.get(0).angle;
            if (rotatedRects.get(0).size.width > rotatedRects.get(0).size.height)
                procAngle *= -1;
            else
                procAngle = 90-procAngle;
            procAngle -= 90;
            if (procAngle < -90) procAngle += 180;
            telemetry.addData("procAngle ", procAngle);
            sampleAngle = procAngle;
        }
        telemetry.addData("sampleAngle", sampleAngle);







        telemetry.update();


        return frame;
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
//        Paint p = new Paint();
//        p.setColor(Color.BLUE);
//        p.setStrokeWidth(4);
//        canvas.drawCircle((float) getPixelsCenter().x, (float) getPixelsCenter().y, 0, p);
//        canvas.drawCircle((float) getPixelsCenter().x, (float) getPixelsCenter().y, 6, p);
    }

    private double getIntersectionArea_bad(RotatedRect rect1, RotatedRect rect2) {
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

    public ArrayList<Point> getOffsets(ArrayList<RotatedRect> input, double scalingFactor) {
        // Note: This method only works when the camera is directly above the samples, looking straight down

        ArrayList<Point> output = new ArrayList<Point>();

        double height = 10.0; // in inches
        double canvasVertical = 1.1 * height*3.0/8.0; // inches
        double canvasHorizontal = 1.1 * height / 2;
        // TODO: Make height not hardcoded, instead base it off of robot position

        double scaled320 = 320/scalingFactor;
        double scaled240 = 240/scalingFactor;
        for (RotatedRect i : input) {
            // real center is (320, 480), positive direction is right and down
//            output.add(new Point(i.center.x - 320, -(i.center.y - 240)));
            output.add(new Point((i.center.x - scaled320) / scaled320 * canvasHorizontal, -(i.center.y - scaled240) / scaled240 * canvasVertical));


            // 4 in height = 1.5 width vertical (half width, not full)
            // 6 : 2.25
            // 2 : 0.75
            // 8 : 3
            // horizontal: 8 / 4
        }

        return output;
    }

    public ArrayList<Point> getOffsets(ArrayList<RotatedRect> input, double height, double scalingFactor) {
        // Note: This method only works when the camera is directly above the samples, looking straight down

        ArrayList<Point> output = new ArrayList<Point>();

        double canvasVertical = 1.1 * height*3.0/8.0; // inches
        double canvasHorizontal = 1.1 * height / 2;

        double scaled320 = 320/scalingFactor;
        double scaled240 = 240/scalingFactor;
        for (RotatedRect i : input) {
            // real center is (320, 480), positive direction is right and down
//            output.add(new Point(i.center.x - 320, -(i.center.y - 240)));
            output.add(new Point((i.center.x - scaled320) / scaled320 * canvasHorizontal, -(i.center.y - scaled240) / scaled240 * canvasVertical));


            // 4 in height = 1.5 width vertical (half width, not full)
            // 6 : 2.25
            // 2 : 0.75
            // 8 : 3
            // horizontal: 8 / 4
        }


        return output;
    }


    private Point from3D(double FOV, double x, double y, double z) {
        return new Point(FOV*x/z, FOV*y/z);
    }

//    private void drawRectangle(double x0, double y0, double z0, double x1, double y1, double z1) {
//        double dx = x1-x0;
//        double dy = y1-y0;
//        double dz = z1-z0;
//        Point p1 = new Point(x0, y0, z0);
//        Point p1 = new Point(x0, y0, z0);
//    }

}