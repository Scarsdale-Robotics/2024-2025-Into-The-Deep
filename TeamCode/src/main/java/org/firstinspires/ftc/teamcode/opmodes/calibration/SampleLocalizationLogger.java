package org.firstinspires.ftc.teamcode.opmodes.calibration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.DetectorResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

@Config
@TeleOp(name="Sample Localization Logger", group="Calibration")
public class SampleLocalizationLogger extends LinearOpMode {

    private Limelight3A limelight;

    // Sample pose estimation coefficients
    public static double k_area = 2.6;

    // Sample pose estimation probability function
    public static double FIELD_RESOLUTION = 1; // inches
    public static double DECAY_TIME = 1;
    private final int RESOLUTION_N = (int) (144.0 / FIELD_RESOLUTION);

    // Array of probabilities (not normalized to 1) of a sample being at that location.
    //
    // .. .. .. ..
    // 30 31 32 33 ...
    // 20 21 22 23 ...
    // 10 11 12 13 ...
    // 00 01 02 03 ...
    //
    // where bottom-left is (-72,-72)in
    private double[][] sample_probability_distribution;

    private ElapsedTime runtime;

    // Localization KF values
    public static double translation_covariance = 0.1;


    @Override
    public void runOpMode() throws InterruptedException {
        this.telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        this.limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Switch to detector
        this.limelight.close();
        this.limelight.pipelineSwitch(2);
        this.limelight.start();

        // Init math

        waitForStart();

        initSampleProbabilityDistribution();

        Set<Color> colors = new HashSet<>(
                Arrays.asList(Color.BLUE, Color.RED, Color.YELLOW)
        );

        while (opModeIsActive()) {

            List<DetectorResult> selectedDetections = getLimelightDetections(limelight, colors);
            updateSampleProbabilityDistribution(new Pose2d(), selectedDetections);

            // draw prob distribution
            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawSampleHeatmap(packet.fieldOverlay(), sample_probability_distribution, FIELD_RESOLUTION, RESOLUTION_N, telemetry);

            if (!selectedDetections.isEmpty()) {
                telemetry.addData("_", "SAMPLE DETECTED");

                // Log each sample
                for (int i = 0; i < selectedDetections.size(); i++) {
                    // Corners
                    DetectorResult detection = selectedDetections.get(i);
                    telemetry.addData(String.format("[%s].getTargetCorners", i), detection.getTargetCorners());

                    // Pose estimation
                    Pose2d samplePoseEstimation = calculateGlobalPosition(new Pose2d(), calculateSampleRelativePosition(detection));
                    telemetry.addData(String.format("[%s]SAMPLE X", i), samplePoseEstimation.getX());
                    telemetry.addData(String.format("[%s]SAMPLE Y", i), samplePoseEstimation.getY());

                    if (gamepad1.cross)
                        Drawing.drawSample(packet.fieldOverlay(), samplePoseEstimation);
                }

            } else {
                telemetry.addData("_", "NO DETECTION");
            }

            telemetry.update();
            FtcDashboard.getInstance().sendTelemetryPacket(packet);


        }



    }





    private void initSampleProbabilityDistribution() {
        sample_probability_distribution = new double[RESOLUTION_N+1][RESOLUTION_N+1];

        runtime = new ElapsedTime(0);
        runtime.reset();
    }


    /**
     * Gets the Limelight's detected samples of the given color.
     * @param limelight
     * @param colors
     * @return a List containing the specified DetectorResults.
     */
    private List<DetectorResult> getLimelightDetections(Limelight3A limelight, Set<Color> colors) {

        LLResult result = limelight.getLatestResult();
        List<DetectorResult> detections = result.getDetectorResults();
        List<DetectorResult> selectedDetections = new ArrayList<>();

        // Filter out samples
        for (DetectorResult detection : detections) {
            String className = detection.getClassName();
            Color detectedColor;
            switch (className) {
                case "red":
                    detectedColor = Color.RED;
                    break;
                case "blue":
                    detectedColor = Color.BLUE;
                    break;
                default:
                    detectedColor = Color.YELLOW;
            }

            if (colors.contains(detectedColor)) selectedDetections.add(detection);
        }

        return selectedDetections;
    }


    /**
     * Estimates the sample's position relative to the robot given the midpoint of its bounding box's top edge.
     *
     * @param detection The Limelight3A detector result for this sample.
     * @return the estimated position.
     */
    private Pose2d calculateSampleRelativePosition(DetectorResult detection) {

        List<List<Double>> corners = detection.getTargetCorners();
        double width = Math.abs(corners.get(1).get(0) - corners.get(0).get(0));
        double height = Math.abs(corners.get(2).get(1) - corners.get(1).get(1));
        double dimensionRatio = width / height;
        double orientationProportion = (2.1538 - dimensionRatio) / 1.2638;
        double orientationMultiplier = 1 - 3.2/12.6*orientationProportion;

        // Get pixel coordinates of the midpoint of the bounding box's top edge.
        double theta_x = -Math.toRadians(detection.getTargetXDegreesNoCrosshair());
        double area = detection.getTargetArea();

        double distance = orientationMultiplier * k_area/Math.sqrt(area);

        return new Pose2d(
                distance * Math.cos(theta_x),
                distance * Math.sin(theta_x),
                new Rotation2d(Math.toRadians(0))
        );
    }


    /**
     * Calculates the global position of an object given its relative pose to the robot and the robot's Pose.
     * @param robotPose
     * @param relativePose
     * @return the object's global Pose2d.
     */
    private Pose2d calculateGlobalPosition(Pose2d robotPose, Pose2d relativePose) {
        double x = relativePose.getX();
        double y = relativePose.getY();
        double dTheta = robotPose.getHeading();
        double cos = Math.cos(dTheta);
        double sin = Math.sin(dTheta);

        return new Pose2d( new Translation2d(
                x*cos - y*sin,
                x*sin + y*cos
        ).plus(robotPose.getTranslation()),
                robotPose.getRotation().plus(relativePose.getRotation())
        );
    }


    /**
     * Update the sample probability distribution array to the next time step.
     * @param detections a List containing the latest sample detections.
     */
    private void updateSampleProbabilityDistribution(Pose2d currentPose, List<DetectorResult> detections) {
        // Exponentially decay current probabilities
        double deltaTime = runtime.seconds();
        runtime.reset();
        double decay = Math.pow(0.5, deltaTime/Math.abs(DECAY_TIME));
        for (int i = 0; i < RESOLUTION_N; i++) {
            for (int j = 0; j < RESOLUTION_N; j++) sample_probability_distribution[i][j] *= decay;
        }

        telemetry.addData("deltaTime", deltaTime);

        // Get pose estimations for each sample
        List<Pose2d> poseEstimations = new ArrayList<>();
        for (DetectorResult detection : detections)
            poseEstimations.add(calculateGlobalPosition(currentPose, calculateSampleRelativePosition(detection)));

        // Update probability distribution array
        for (Pose2d pose : poseEstimations) {
            double x = pose.getX();
            double y = pose.getY();

            for (int r = 0; r < RESOLUTION_N+1; r++) {
                for (int c = 0; c < RESOLUTION_N+1; c++) {
                    double px = FIELD_RESOLUTION * (c - (double)RESOLUTION_N/2);
                    double py = FIELD_RESOLUTION * (r - (double)RESOLUTION_N/2);
                    sample_probability_distribution[r][c] += bivariateNormalDistribution(px, py, x, y, translation_covariance);
                }
            }
        }

        // Normalize probability distribution array
        double maxProbability = 0;
        double[] maxProbabilityCoordinates = new double[]{-99,-99};
        for (int r = 0; r < RESOLUTION_N+1; r++) {
            for (int c = 0; c < RESOLUTION_N+1; c++) {
                // telemetry -- remove later
                if (sample_probability_distribution[r][c] > maxProbability) {
                    maxProbability = sample_probability_distribution[r][c];
                    double px = FIELD_RESOLUTION * (c - (double)RESOLUTION_N/2);
                    double py = FIELD_RESOLUTION * (r - (double)RESOLUTION_N/2);
                    maxProbabilityCoordinates = new double[]{px,py};
                }
                if (sample_probability_distribution[r][c] > 1) {
                    sample_probability_distribution[r][c] = 1;
                }
            }
        }
        telemetry.addData("[PROB] maxProbability", maxProbability);
        telemetry.addData("[PROB] maxProbabilityCoordinates", String.format("(%s,%s)",maxProbabilityCoordinates[0], maxProbabilityCoordinates[1]));


    }


    /**
     * <a href="https://en.wikipedia.org/wiki/Multivariate_normal_distribution">...</a>
     */
    private double bivariateNormalDistribution(double x, double y, double mean_x, double mean_y, double covariance) {
        return Math.exp(-0.5 * (Math.pow(x-mean_x, 2) + Math.pow(y-mean_y, 2)) / covariance)
                / (2*Math.PI*covariance);
    }





    public enum Color { RED, YELLOW, BLUE }

}
