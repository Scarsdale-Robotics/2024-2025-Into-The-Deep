package org.firstinspires.ftc.teamcode.synchropather.systems.limelight;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotSystem;
import org.firstinspires.ftc.teamcode.opmodes.calibration.Drawing;
import org.firstinspires.ftc.teamcode.synchropather.systems.MovementType;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.Movement;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.Plan;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

@Config
public class LimelightPlan extends Plan<LimelightState> {

    private final RobotSystem robot;
    private final Limelight3A limelight;

    private LimelightState lastState;


    /**
     * Creates a new LimelightPlan with the given parameters
     * @param robot the RobotSystem.
     * @param colorToDetect the color of the samples to detect.
     * @param movements
     */
    public LimelightPlan(RobotSystem robot, Color colorToDetect, Movement... movements) {
        super(MovementType.LIMELIGHT, movements);
        this.robot = robot;
        this.limelight = robot.cv.getLimelight();
        this.lastState = null;
        initSampleDetector(colorToDetect);
    }

    @Override
    public void loop() {
        LimelightState currentState = getCurrentState();

        // Check if any state components have changed since last loop
        boolean enabledChanged = false;
        boolean pipelineChanged = false;
        if (lastState == null) {
            enabledChanged = true;
            pipelineChanged = true;
        } else if (lastState.getEnabled() != currentState.getEnabled()) {
            enabledChanged = true;
        } else if (!lastState.getPipeline().equals(currentState.getPipeline())) {
            pipelineChanged = true;
        }

        // Handle enabling/disabling
        if (enabledChanged) {
            if (currentState.getEnabled()) {
                limelight.start();
            } else {
                limelight.pause();
            }
        }

        // Handle pipeline switching
        if (pipelineChanged) {
            limelight.pipelineSwitch(currentState.getPipeline().pipelineIndex);
            limelight.reloadPipeline();
        }


        // Pipeline specific actions
        if (limelight.isRunning() && currentState.getEnabled()) {
            if (currentState.getPipeline() == LimelightPipeline.SAMPLE_DETECTOR) loopSampleDetector();
        }


        lastState = currentState;
    }

    @Override
    public void stop() {
        limelight.close();
    }









    ///////////////////////////
    // DETECTOR PIPELINE (2) //
    ///////////////////////////
    // Sample pose estimation coefficients
    public static double theta_incline = 0; // radians
    public static double k1 = 480;
    public static double k2 = 640;
    public static double cz = 3.625; // inches from above the field
    public static double dist_covariance = 0.25;

    // Sample pose estimation probability function
    public static double FIELD_RESOLUTION = 1; // inches
    public static double DECAY_TIME = 0.5;
    private final int RESOLUTION_N = (int) (144.0 / FIELD_RESOLUTION);

    public static double SAMPLE_PROBABILITY_THRESHOLD = 3;
    public static double SAMPLE_CLUSTER_SIZE_THRESHOLD = 0.5; // inches^2

    private ElapsedTime runtime;

    // METHODS //
    private void initSampleDetector(Color colorToDetect) {
        robot.cv.sample_probability_distribution = new double[RESOLUTION_N+1][RESOLUTION_N+1];
        robot.cv.detectedSamples = new ArrayList<>();

        runtime = new ElapsedTime(0);
        runtime.reset();

        robot.cv.desiredColors = new HashSet<>(
                Collections.singletonList(colorToDetect)
        );
    }

    private void loopSampleDetector() {
        List<LLResultTypes.DetectorResult> selectedDetections = getLimelightDetections(limelight, robot.cv.desiredColors);
        updateSampleProbabilityDistribution(robot.localization.getPose(), selectedDetections);

        // draw each calculated sample
        if (robot.opMode.gamepad1.circle) {
            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            for (Pose2d detectedSample : robot.cv.detectedSamples) {
                Drawing.drawSample(packet.fieldOverlay(), detectedSample, "#90b270");
            }
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }

    /**
     * Gets the Limelight's detected samples of the given color.
     * @param limelight
     * @param colors
     * @return a List containing the specified DetectorResults.
     */
    private List<LLResultTypes.DetectorResult> getLimelightDetections(Limelight3A limelight, Set<Color> colors) {

        LLResult result = limelight.getLatestResult();
        List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();
        List<LLResultTypes.DetectorResult> selectedDetections = new ArrayList<>();

        // Filter out samples
        for (LLResultTypes.DetectorResult detection : detections) {
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
    private Pose2d calculateSampleRelativePosition(LLResultTypes.DetectorResult detection, double k1, double k2, double cz, double theta_incline) {

        // math: https://www.desmos.com/calculator/hvfwopk7tw

        // Get pixel coordinates of midpoint of top edge of bounding box.
        List<List<Double>> corners = detection.getTargetCorners();
        double px = 0.5 * (corners.get(0).get(0) + corners.get(1).get(0));
        double py = corners.get(0).get(1);

        // Back-solve for projected coordinates
        double c_px = (px - 320) / k2;
        double c_py = (py - 240) / k1;

        double projectedX_numer = -c_py*Math.sin(theta_incline) - Math.cos(theta_incline);
        double projectedX_denom = c_py*Math.cos(theta_incline) - Math.sin(theta_incline);
        if (projectedX_denom == 0) return null;

        double projectedX = cz * projectedX_numer / projectedX_denom;
        double projectedY = -(projectedX*Math.cos(theta_incline) + cz*Math.sin(theta_incline)) * c_px;

        return new Pose2d(
                projectedX,
                projectedY,
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
    private void updateSampleProbabilityDistribution(Pose2d currentPose, List<LLResultTypes.DetectorResult> detections) {

        // Get pose estimations for each sample
        List<Pose2d> poseEstimations = new ArrayList<>();
        for (LLResultTypes.DetectorResult detection : detections) {
            Pose2d relativePosition = calculateSampleRelativePosition(detection, k1, k2, cz, theta_incline);
            if (relativePosition != null) {
                poseEstimations.add(calculateGlobalPosition(currentPose, relativePosition));
            }
        }


        // Update probability distribution array
        for (Pose2d pose : poseEstimations) {
            double x = pose.getX();
            double y = pose.getY();

            // Only update within a neighborhood of SD < 3.
            double C = 2*Math.PI* dist_covariance;
            double T = 0.05; // probability threshold
            int d = (int)Math.ceil(Math.sqrt(Math.max(0, -C*Math.log(C*T))));

            int rMin = (int)Math.max(0, (y-d)/FIELD_RESOLUTION + (double)RESOLUTION_N/2);
            int rMax = (int)Math.min(RESOLUTION_N+1, (y+d)/FIELD_RESOLUTION + (double)RESOLUTION_N/2);
            for (int r = rMin; r < rMax; r++) {
                int cMin = (int)Math.max(0, (x-d)/FIELD_RESOLUTION + (double)RESOLUTION_N/2);
                int cMax = (int)Math.min(RESOLUTION_N+1, (x+d)/FIELD_RESOLUTION + (double)RESOLUTION_N/2);
                for (int c = cMin; c < cMax; c++) {
                    double px = FIELD_RESOLUTION * (c - (double)RESOLUTION_N/2);
                    double py = FIELD_RESOLUTION * (r - (double)RESOLUTION_N/2);
                    robot.cv.sample_probability_distribution[r][c] += bivariateNormalDistribution(px, py, x, y, dist_covariance);
                }
            }
        }


        // Find potential sample indices
        Map<Integer, Boolean> visited = new HashMap<>();
        ArrayDeque<Integer> availableStartingPositions = new ArrayDeque<>();

        // Exponentially decay current probabilities
        double deltaTime = runtime.seconds();
        runtime.reset();
        // Half life formula
        double decay = Math.pow(0.5, deltaTime/Math.abs(DECAY_TIME));
        // Decay by stepDistance^2 neighborhood (saves processing power)
        double stepDistance = 1d;
        int step = (int)(stepDistance/FIELD_RESOLUTION);
        int start = (int)(step*Math.random());
        for (int i = start; i < RESOLUTION_N+1; i+=step) {
            for (int j = start; j < RESOLUTION_N+1; j+=step) {
                // only decay block if there is a significant value (P>0.05)
                if (robot.cv.sample_probability_distribution[i][j]>0.05){
                    for (int i1 = i-start; i1 < Math.min(RESOLUTION_N+1, i-start+step); i1++) {
                        for (int j1 = j-start; j1 < Math.min(RESOLUTION_N+1, j-start+step); j1++) {
                            robot.cv.sample_probability_distribution[i1][j1] *= decay;
                            // Check if current index is valid for sample detection
                            if (robot.cv.sample_probability_distribution[i1][j1]>=SAMPLE_PROBABILITY_THRESHOLD) {
                                int shortenedIndex = i1*(RESOLUTION_N+1) + j1;
                                visited.put(shortenedIndex, false);
                                availableStartingPositions.add(shortenedIndex);
                            }
                        }
                    }
                }
            }
        }
        robot.telemetry.addData("deltaTime", deltaTime);


        // FF to find samples (center of mass of clusters)
        ArrayList<Pose2d> detectedSamples = new ArrayList<>();
        ArrayDeque<Integer> q = new ArrayDeque<>();
        int[] di = new int[]{-(RESOLUTION_N+1),(RESOLUTION_N+1),-1,1};
        while (!availableStartingPositions.isEmpty()) {
            // Loop through available valid starting positions
            int check = availableStartingPositions.removeFirst();

            // Found a cluster
            if (Boolean.FALSE.equals(visited.get(check))) {
                double clusterSize = 0;
                double dottedX = 0;
                double dottedY = 0;
                double totalWeight = 0;

                q.add(check);
                visited.put(check, true);

                // Compute the center of mass of the cluster
                while (!q.isEmpty()) {
                    // Get first element in queue
                    int i0 = q.removeFirst();
                    visited.put(i0, true);

                    // Decode coordinates
                    int r0 = i0 / (RESOLUTION_N+1);
                    int c0 = i0 % (RESOLUTION_N+1);
                    double weight = robot.cv.sample_probability_distribution[r0][c0];
                    double x0 = FIELD_RESOLUTION * (c0 - (double) RESOLUTION_N / 2);
                    double y0 = FIELD_RESOLUTION * (r0 - (double) RESOLUTION_N / 2);

                    // Update cluster information
                    clusterSize++;
                    dottedX += x0*weight;
                    dottedY += y0*weight;
                    totalWeight += weight;

                    // Check neighbors
                    for (int dir = 0; dir < 4; dir++) {
                        int i1 = i0+di[dir];
                        if (Boolean.FALSE.equals(visited.get(i1))) {
                            q.add(i1);
                        }
                    }
                }

                // Record center of mass
                if (clusterSize > SAMPLE_CLUSTER_SIZE_THRESHOLD/(Math.pow(FIELD_RESOLUTION,2))) {
                    dottedX /= totalWeight;
                    dottedY /= totalWeight;
                    detectedSamples.add(new Pose2d(
                            dottedX,
                            dottedY,
                            new Rotation2d()
                    ));
                }
            }
        }
        robot.cv.detectedSamples = detectedSamples;


        // telemetry -- remove later
        if (robot.opMode.gamepad1.square) {
            double maxProbability = 0;
            double[] maxProbabilityCoordinates = new double[]{-99, -99};
            for (int r = 0; r < RESOLUTION_N + 1; r++) {
                for (int c = 0; c < RESOLUTION_N + 1; c++) {
                    if (robot.cv.sample_probability_distribution[r][c] > maxProbability) {
                        maxProbability = robot.cv.sample_probability_distribution[r][c];
                        double px = FIELD_RESOLUTION * (c - (double) RESOLUTION_N / 2);
                        double py = FIELD_RESOLUTION * (r - (double) RESOLUTION_N / 2);
                        maxProbabilityCoordinates = new double[]{px, py};
                    }
                }
            }
            robot.telemetry.addData("[PROB] maxProbability", maxProbability);
            robot.telemetry.addData("[PROB] maxProbabilityCoordinates", String.format("(%s,%s)", maxProbabilityCoordinates[0], maxProbabilityCoordinates[1]));
        }

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
