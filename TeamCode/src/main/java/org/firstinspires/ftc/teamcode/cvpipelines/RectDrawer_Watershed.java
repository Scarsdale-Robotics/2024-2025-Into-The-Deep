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
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

public class RectDrawer_Watershed extends OpenCvPipeline {

    public enum SampleColor {
        YELLOW(),
        BLUE(),
        RED();
    }

    public Mat frame;

    public static Telemetry telemetry;

    public static Scalar lowerYellow = new Scalar(15.0, 100.0, 80.1); // hsv
    public static Scalar upperYellow = new Scalar(30.0, 255.0, 255.0); // hsv
    public static Scalar lowerBlue = new Scalar(90.0, 150.0, 60.0); // hsv
    public static Scalar upperBlue = new Scalar(140.0, 255.0, 255.0); // hsv
    public static Scalar lowerRedH = new Scalar(10.0, 0.0, 0.0); // hsv
    public static Scalar upperRedH = new Scalar(160.0, 255.0, 255.0); // hsv
    public static Scalar lowerRedSV = new Scalar(0.0, 190.0, 120.0); // hsv
    public static Scalar upperRedSV = new Scalar(255.0, 255.0, 255.0); // hsv

    private double sampleAngle = 0;

    public static SampleColor colorType = SampleColor.YELLOW;
//    @Override
//    public void init(int width, int height, CameraCalibration calibration) {
//
//    }

    public RectDrawer_Watershed(Telemetry telemetry){
        this.telemetry = telemetry;
//        this.colorType = colorType;
    }

    @Override
    public Mat processFrame(Mat input) {
        frame = input.clone();
        double scalingFactor = (double) 640 /frame.width();

//        if (true) return frame;


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

        Mat binaryImage = inRange.clone();


        // Convert to RGB (Watershed requires a 3-channel image)
        Mat markers = new Mat();
        Imgproc.cvtColor(binaryImage, markers, Imgproc.COLOR_GRAY2BGR);

        // Apply morphological opening to remove noise
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
        Imgproc.morphologyEx(binaryImage, binaryImage, Imgproc.MORPH_OPEN, kernel);

        // Compute distance transform
        Mat distTransform = new Mat();
        Imgproc.distanceTransform(binaryImage, distTransform, Imgproc.DIST_L2, 5);
        Core.normalize(distTransform, distTransform, 0, 255, Core.NORM_MINMAX);

        // Threshold to get sure foreground
        Mat sureForeground = new Mat();
        Imgproc.threshold(distTransform, sureForeground, 0.7 * 255, 255, Imgproc.THRESH_BINARY);
        sureForeground.convertTo(sureForeground, CvType.CV_8U);  // Ensure correct type

        // Get sure background by dilating the original thresholded image
        Mat sureBackground = new Mat();
        Imgproc.dilate(binaryImage, sureBackground, kernel);
        sureBackground.convertTo(sureBackground, CvType.CV_8U);  // Ensure correct type

        // Find unknown region
        Mat unknown = new Mat();
        Core.subtract(sureBackground, sureForeground, unknown);

        // Label markers for watershed
        Mat markers32 = new Mat();
        sureForeground.convertTo(markers32, CvType.CV_32S);  // Must be CV_32S
        Imgproc.watershed(markers, markers32);

        // Convert negative values to 255 (boundary markers)
        // For object regions (positive markers), apply a color mapping.
        Core.subtract(markers32, new Scalar(1), markers32); // Ensure positive labeling for watershed regions

        // Normalize markers for visualization
        markers32.convertTo(markers32, CvType.CV_8U); // Convert to 8-bit for color mapping
        Mat result = new Mat();
        Imgproc.applyColorMap(markers32, result, Imgproc.COLORMAP_JET);  // Use JET color map for objects

        // Add a red color to the boundary regions
        // Create a boundary mask for the regions marked as boundaries (negative values).
        Mat boundaryMask = new Mat(markers32.size(), CvType.CV_8U, new Scalar(0));
        markers32.convertTo(boundaryMask, CvType.CV_8U, 1, 0); // Convert negative markers to 255 (boundary mask)

        // Set boundary regions to red
        Core.add(result, new Scalar(0, 0, 255), result);  // Red boundaries


        distTransform.convertTo(distTransform, CvType.CV_8U); // Convert to 8-bit for visualization
        markers32.convertTo(markers32, CvType.CV_8U); // Convert to 8-bit for visualization
        if (true) return result;


        // Step 1: Apply Canny Edge Detection
        Mat edges = new Mat();
        Imgproc.Canny(inRange, edges, 50, 150);

        // Step 2: Detect lines using Hough Line Transform
        Mat lines = new Mat();
        Imgproc.HoughLinesP(edges, lines, 1, Math.PI / 180, 11, 0, 40);

        Mat rgbImage = new Mat();

        // Convert single-channel binary image to 3-channel RGB
        Imgproc.cvtColor(edges, rgbImage, Imgproc.COLOR_GRAY2BGR);

        // Step 3: Draw detected lines and analyze them
//        Mat lineImage = new Mat(inRange.size(), CvType.CV_8UC3, new Scalar(0, 0, 0));
        for (int i = 0; i < lines.rows(); i++) {
            double[] line = lines.get(i, 0);
            Point pt1 = new Point(line[0], line[1]);
            Point pt2 = new Point(line[2], line[3]);

            // Draw the detected lines
            Imgproc.line(inRange, pt1, pt2, new Scalar(0, 0, 0), 2);
            Imgproc.line(rgbImage, pt1, pt2, new Scalar(0, 0, 255), 2);
        }

        if (true) return rgbImage;

//        int n = 5;
//        Imgproc.erode(inRange, inRange, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(n, n)));

//        if (true) return inRange;


        // Find all contours
        List<MatOfPoint> unfilteredContours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(inRange, unfilteredContours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        // Filter contours by size and get rotated rects
        int minArea = 1250;
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

        // telemetry
        ArrayList<ArrayList<Double>> overlapGroups2 = new ArrayList<>();
        for (ArrayList<Double[]> overlapGroup : overlapGroups) {
            overlapGroups2.add(new ArrayList<>());
            for (Double[] index : overlapGroup) {
                overlapGroups2.get(overlapGroups2.size()-1).add(index[0]);
            }
        }
        telemetry.addData("overlapGroups", overlapGroups2);

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

        telemetry.addData("filteredRects.size()", filteredRects.size());


//        // Draw unfiltered rects as blue
//        for (RotatedRect rotatedRect : rotatedRects) {
//            Point[] vertices = new Point[4];
//            rotatedRect.points(vertices);
//            for (int i = 0; i < 4; i++) {
//                Imgproc.line(frame, vertices[i], vertices[(i + 1) % 4], new Scalar(0, 0, 255), 2);
//            }
//        }




        // telemetry
        if (!filteredRects.isEmpty()) {
            telemetry.addData("width ", filteredRects.get(0).size.width);
            telemetry.addData("height ", filteredRects.get(0).size.height);
            telemetry.addData("area ", filteredRects.get(0).size.area());
            telemetry.addData("angle ", filteredRects.get(0).angle);
            telemetry.addData("center ", filteredRects.get(0).center);
//            telemetry.addData("center scaled", new Point((filteredRects.get(0).center.x - 320) / 640 * 3.0/8.0, -(filteredRects.get(0).center.y - 240) / 480));
//            output.add(new Point((i.center.x - 320) / 640 * canvasHorizontal, -(i.center.y - 240) / 240 * canvasVertical));
            double procAngle = filteredRects.get(0).angle;
            if (filteredRects.get(0).size.width > filteredRects.get(0).size.height)
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

//        if (true) return frame;




        // Draw filtered rects as green and their corresponding data
        ArrayList<Point> real = getOffsets(filteredRects, scalingFactor);
        for (int i = 0; i < filteredRects.size(); i++) {
            RotatedRect rotatedRect = filteredRects.get(i);
            Point[] vertices = new Point[4];
            rotatedRect.points(vertices);
            for (int j = 0; j < 4; j++) {
                Imgproc.line(frame, vertices[j], vertices[(j + 1) % 4], new Scalar(0, 255, 0), 2);
            }
            Point center = rotatedRect.center;



            double procAngle = rotatedRect.angle;
            if (filteredRects.get(0).size.width > filteredRects.get(0).size.height)
                procAngle *= -1;
            else
                procAngle = 90-procAngle;

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
            real = getOffsets(filteredRects, sampleHeight, scalingFactor);






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

        telemetry.update();


        return frame;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
//        Paint p = new Paint();
//        p.setColor(Color.BLUE);
//        p.setStrokeWidth(4);
//        canvas.drawCircle((float) getPixelsCenter().x, (float) getPixelsCenter().y, 0, p);
//        canvas.drawCircle((float) getPixelsCenter().x, (float) getPixelsCenter().y, 6, p);
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