package org.firstinspires.ftc.teamcode.cvprocessors;

import android.graphics.Canvas;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ConcurrentLinkedDeque;
import java.util.concurrent.CopyOnWriteArrayList;

@Config
public class LimelightDetectorProcessor implements VisionProcessor {

    public static boolean SHOW_SAMPLE_DISTRIBUTION = true;
    public static double FIELD_RESOLUTION = 0.5; // inches
    public static double DECAY_TIME = 0.5;
    public static final int RESOLUTION_N = (int) (144.0 / FIELD_RESOLUTION);
    public static double DETECTION_COVARIANCE = 0.25;
    public static double PEAK_THRESHOLD = 0.9;


    public Mat yellowDistribution;
    public Mat blueDistribution;
    public Mat redDistribution;
    private final Mat gaussian2D;
    private int ksize;

    private List<Point> yellowProbabilityPeaks;
    private List<Point> blueProbabilityPeaks;
    private List<Point> redProbabilityPeaks;

    private final ConcurrentLinkedDeque<double[]> samplePositionsToProcess;
    private CopyOnWriteArrayList<double[]> yellowSamplePositions;
    private CopyOnWriteArrayList<double[]> blueSamplePositions;
    private CopyOnWriteArrayList<double[]> redSamplePositions;


    public LimelightDetectorProcessor() {
        // Initialize an n x n single-channel matrix filled with zeros
        yellowDistribution = Mat.zeros(RESOLUTION_N, RESOLUTION_N, CvType.CV_64F);
        blueDistribution = Mat.zeros(RESOLUTION_N, RESOLUTION_N, CvType.CV_64F);
        redDistribution = Mat.zeros(RESOLUTION_N, RESOLUTION_N, CvType.CV_64F);

        // Initialize gaussian dist
        double sigma = Math.sqrt(DETECTION_COVARIANCE)/FIELD_RESOLUTION;  // Standard deviation for Gaussian
        ksize = (int) Math.max(3, 6 * sigma);  // Kernel size (typically 6*sigma for near-zero tail)
        if (ksize % 2 == 0) ksize += 1;  // Ensure odd size for symmetry

        // Generate 1D Gaussian Kernels
        Mat kernelX = Imgproc.getGaussianKernel(ksize, sigma, CvType.CV_64F);
        Mat kernelY = Imgproc.getGaussianKernel(ksize, sigma, CvType.CV_64F);

        // Compute 2D Gaussian by outer product
        gaussian2D = new Mat();
        Core.gemm(kernelY, kernelX.t(), 1, new Mat(), 0, gaussian2D); // Outer product
        Core.normalize(gaussian2D, gaussian2D, 0, 0.5, Core.NORM_MINMAX);

        // Initialize sample position data storage
        samplePositionsToProcess = new ConcurrentLinkedDeque<>();

        yellowSamplePositions = new CopyOnWriteArrayList<>();
        yellowProbabilityPeaks = new ArrayList<>();

        blueSamplePositions = new CopyOnWriteArrayList<>();
        blueProbabilityPeaks = new ArrayList<>();

        redSamplePositions = new CopyOnWriteArrayList<>();
        redProbabilityPeaks = new ArrayList<>();
    }


    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }


    @Override
    public Object processFrame(Mat input, long captureTimeNanos) {
        if (!samplePositionsToProcess.isEmpty()) {
            // Decay current probabilities
            double deltaTime = 1.0/30.0;
            // Half life formula
            double decay = Math.pow(0.5, deltaTime/Math.abs(DECAY_TIME));
            yellowDistribution.convertTo(yellowDistribution, -1, decay, 0);
            blueDistribution.convertTo(blueDistribution, -1, decay, 0);
            redDistribution.convertTo(redDistribution, -1, decay, 0);

            // Process new positions
            processSamplePositions();

            // Update local maxima
            updateLocalMaxima();
        }

        // Visual telemetry
        if (SHOW_SAMPLE_DISTRIBUTION) {
            // Normalize to 8-bit range for display
            Mat viewable = new Mat();
            Core.add(yellowDistribution, blueDistribution, viewable);
            Core.add(viewable, redDistribution, viewable);
            Core.normalize(viewable, viewable, 0, 255, Core.NORM_MINMAX);
            viewable.convertTo(viewable, CvType.CV_8U);  // Convert to 8-bit grayscale
            Imgproc.cvtColor(viewable, viewable, Imgproc.COLOR_GRAY2BGR);
            for (Point peak : yellowProbabilityPeaks) Imgproc.circle(viewable,peak,1,new Scalar(255,255,0),-1);
            for (Point peak : blueProbabilityPeaks) Imgproc.circle(viewable,peak,1,new Scalar(0,0,255),-1);
            for (Point peak : redProbabilityPeaks) Imgproc.circle(viewable,peak,1,new Scalar(255,0,0),-1);
            return viewable;
        }
        return input;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    /**
     * Sample Color Index:
     * 0: Yellow
     * 1: Blue
     * 2: Red
     * @param samplePosition {x, y, sample color}
     */
    public void addSamplePosition(double[] samplePosition) {
        if (samplePosition.length!=3) return;
        samplePositionsToProcess.add(samplePosition);
    }

    public List<double[]> getYellowSamplePositions() {
        return new CopyOnWriteArrayList<>(yellowSamplePositions);
    }

    public List<double[]> getBlueSamplePositions() {
        return new CopyOnWriteArrayList<>(blueSamplePositions);
    }

    public List<double[]> getRedSamplePositions() {
        return new CopyOnWriteArrayList<>(redSamplePositions);
    }

    private void processSamplePositions() {
        while (!samplePositionsToProcess.isEmpty()) {
            double[] samplePosition = samplePositionsToProcess.pollFirst();
            int c = RESOLUTION_N/2 + (int)(samplePosition[0]/FIELD_RESOLUTION);
            int r = RESOLUTION_N/2 + (int)(samplePosition[1]/FIELD_RESOLUTION);
            if (r<0 || RESOLUTION_N-1<r || c<0 || RESOLUTION_N-1<c) continue;

            // Manual switch color cases
            // 0: yellow
            // 1: blue
            // 2: red
            int sampleColor = (int)samplePosition[2];
            if (sampleColor==0) {
                addGaussianToMat(yellowDistribution, r, c);
            }
            if (sampleColor==1) {
                addGaussianToMat(blueDistribution, r, c);
            }
            if (sampleColor==2) {
                addGaussianToMat(redDistribution, r, c);
            }
        }
    }

    public void addGaussianToMat(Mat mat, int r0, int c0) {
        // Define the region to add the Gaussian
        int rStart = Math.max(0, r0 - ksize / 2);
        int rEnd = Math.min(mat.rows(), r0 + ksize / 2 + 1);
        int cStart = Math.max(0, c0 - ksize / 2);
        int cEnd = Math.min(mat.cols(), c0 + ksize / 2 + 1);

        // Compute corresponding submatrix in the Gaussian kernel
        int gRStart = rStart - (r0 - ksize / 2);  // Shifted row start in gaussian2D
        int gREnd = gRStart + (rEnd - rStart);    // Match size of the region in mat
        int gCStart = cStart - (c0 - ksize / 2);  // Shifted col start in gaussian2D
        int gCEnd = gCStart + (cEnd - cStart);    // Match size of the region in mat

        Mat roi = mat.submat(rStart, rEnd, cStart, cEnd);
        Mat subGaussian = gaussian2D.submat(gRStart, gREnd, gCStart, gCEnd);
        Core.add(roi, subGaussian, roi);
        Core.min(mat, new Scalar(1), mat);
    }

    private void updateLocalMaxima() {
        CopyOnWriteArrayList<double[]> temp;

        // Find yellow local maxima and replace list
        yellowProbabilityPeaks = findLocalMaxima(yellowDistribution, PEAK_THRESHOLD);
        temp = new CopyOnWriteArrayList<>();
        for (Point peak : yellowProbabilityPeaks) {
            double x_sample = FIELD_RESOLUTION * (peak.x - (double)RESOLUTION_N/2);
            double y_sample = FIELD_RESOLUTION * (peak.y - (double)RESOLUTION_N/2);
            temp.add(new double[]{x_sample, y_sample});
        }
        yellowSamplePositions = temp;

        // Find blue local maxima and replace list
        blueProbabilityPeaks = findLocalMaxima(blueDistribution, PEAK_THRESHOLD);
        temp = new CopyOnWriteArrayList<>();
        for (Point peak : blueProbabilityPeaks) {
            double x_sample = FIELD_RESOLUTION * (peak.x - (double)RESOLUTION_N/2);
            double y_sample = FIELD_RESOLUTION * (peak.y - (double)RESOLUTION_N/2);
            temp.add(new double[]{x_sample, y_sample});
        }
        blueSamplePositions = temp;

        // Find red local maxima and replace list
        redProbabilityPeaks = findLocalMaxima(redDistribution, PEAK_THRESHOLD);
        temp = new CopyOnWriteArrayList<>();
        for (Point peak : redProbabilityPeaks) {
            double x_sample = FIELD_RESOLUTION * (peak.x - (double)RESOLUTION_N/2);
            double y_sample = FIELD_RESOLUTION * (peak.y - (double)RESOLUTION_N/2);
            temp.add(new double[]{x_sample, y_sample});
        }
        redSamplePositions = temp;
    }

    public List<Point> findLocalMaxima(Mat img, double threshold) {
        Mat binary = new Mat();
        Imgproc.threshold(img, binary, threshold, 255, Imgproc.THRESH_BINARY);
        binary.convertTo(binary, CvType.CV_8UC1);  // Convert to 8-bit

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(binary, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        List<Point> peaks = new ArrayList<>();
        for (MatOfPoint contour : contours) {
            Moments m = Imgproc.moments(contour);
            if (m.m00 != 0) {
                int cx = (int) (m.m10 / m.m00);
                int cy = (int) (m.m01 / m.m00);
                peaks.add(new Point(cx, cy));
            }
        }
        return peaks;
    }

}
