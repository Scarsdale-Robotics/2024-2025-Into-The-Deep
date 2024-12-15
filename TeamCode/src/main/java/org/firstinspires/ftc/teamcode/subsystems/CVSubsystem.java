package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.synchropather.systems.limelight.LimelightPlan;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Point;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;


public class CVSubsystem extends SubsystemBase {

    /**
     * PIPELINE INDICES:
     * 0 = AprilTag Pipeline
     */
    private final Limelight3A limelight;


    private final Telemetry telemetry;

    private SampleColor currentTargetColor;

    public final Size CAM_SZ = new Size(640, 480);



    /////////////////////
    // SAMPLE DETECTOR //
    /////////////////////

    public Set<LimelightPlan.Color> desiredColors;
    // Array of probabilities (not normalized to 1) of a sample being at that location.
    //
    // .. .. .. ..
    // 30 31 32 33 ...
    // 20 21 22 23 ...
    // 10 11 12 13 ...
    // 00 01 02 03 ...
    //
    // where bottom-left is (-72,-72)in
    public double[][] sample_probability_distribution;
    public ArrayList<Pose2d> detectedSamples;



    //////////////////
    // INIT METHODS //
    //////////////////

    public CVSubsystem(Limelight3A limelight, double initialHeading, boolean isRedTeam, Telemetry telemetry) {
        this.limelight = limelight;
        this.limelight.updateRobotOrientation(Math.toDegrees(initialHeading));
        this.telemetry = telemetry;
  
        // Switch to AprilTag
//        this.limelight.pipelineSwitch(0);
//        this.limelight.start();

        currentTargetColor = isRedTeam ? SampleColor.RY : SampleColor.BY;

    }



    //////////////////
    // MAIN METHODS //
    //////////////////

    /**
     * Updates the Limelight's heading for the MT2 algorithm.
     * @param heading The robot heading in radians.
     */
    public void updateHeading(double heading) {
        limelight.updateRobotOrientation(Math.toDegrees(heading));
    }

    /**
     * Corrects MegaTag2 pose estimation for constant error and returns a PoseEstimation with covariances.
     * Data for covariances: https://docs.google.com/spreadsheets/d/1GIMsalUpOIxMDBMFBM2qNpxxl2moQsIOBuP_30yjZfc/edit?usp=sharing
     * @param result
     * @return the processed PoseEstimation.
     */
    public PoseEstimation processMT2Position(LLResult result) {
        Position position = result.getBotpose_MT2().getPosition().toUnit(DistanceUnit.INCH);
        YawPitchRollAngles orientation = result.getBotpose_MT2().getOrientation();

        // Correct for constant error
        double heading = orientation.getYaw(AngleUnit.RADIANS);
        double distanceToTag = Double.MAX_VALUE;
        double tagYaw = -1;
        List<List<Double>> tagCorners = new ArrayList<>();
        for (LLResultTypes.FiducialResult fiducialResult : result.getFiducialResults()) {
            Position tagPosition = fiducialResult.getRobotPoseTargetSpace().getPosition().toUnit(DistanceUnit.INCH);
            double distance = Math.hypot(tagPosition.x, Math.hypot(tagPosition.y, tagPosition.z));
            distanceToTag = Math.min(distanceToTag, distance);
            tagYaw = fiducialResult.getRobotPoseTargetSpace().getOrientation().getPitch();
            tagCorners = fiducialResult.getTargetCorners();
        }
        telemetry.addData("distanceToTag", distanceToTag);
        telemetry.addData("tagYaw", tagYaw);
        telemetry.addData("tagCorners", tagCorners.toString());
        double estimatedError = 3.67e-3*distanceToTag + 1.34;

        double dx = estimatedError * Math.cos(heading);
        double dy = estimatedError * Math.sin(heading);
        double newX = position.x+dx;
        double newY = position.y+dy;
        double newHeading = result.getBotpose().getOrientation().getYaw(AngleUnit.RADIANS);
        Pose2d pose = new Pose2d(newX, newY, new Rotation2d(newHeading));

        // Calculate translation & rotation covariances
        double translationCovariance = Double.MAX_VALUE;
        if (distanceToTag <= 132) translationCovariance = 0.2*Math.exp(0.0271*distanceToTag) - 0.19;
        translationCovariance *= translationCovariance;

        double rotationCovariance = Double.MAX_VALUE;
        double yawThreshold = 45; // Degrees
        if (distanceToTag <= 36 && Math.abs(tagYaw) < yawThreshold) rotationCovariance = 0.006 + 1.07e-4*Math.exp(0.266*distanceToTag);
        rotationCovariance *= rotationCovariance;

        return new PoseEstimation(pose, translationCovariance, rotationCovariance);
    }

    /**
     * Logs the robot poses calculated using the detected AprilTags relative to the <a href="https://ftc-docs.firstinspires.org/en/latest/game_specific_resources/field_coordinate_system/field-coordinate-system.html#reference-frame">field reference frame</a>.
     */
    public void telemeterAbsoluteAprilTags() {
        LLResult result = limelight.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                PoseEstimation poseEstimation = processMT2Position(result);
                Pose2d pose = poseEstimation.pose;
                telemetry.addData(
                        "LL Botpose",
                        String.format("(X:%s, Y:%s)", pose.getX(), pose.getY())
                );
                telemetry.addData("Translation cov", poseEstimation.translationCovariance);
                telemetry.addData("Rotation cov", poseEstimation.headingCovariance);

            }
        }
        telemetry.update();
    }

    /**
     * Chooses the AprilTag pose estimation with calculated covariances.
     */
    public PoseEstimation getPoseEstimation() {
        LLResult result = limelight.getLatestResult();
        if (result == null) return null;
        if (!result.isValid()) return null;
        return processMT2Position(result);
    }

    public double[] getStddev() {
        LLResult result = limelight.getLatestResult();
        if (result == null) return new double[3];
        if (!result.isValid()) return new double[3];

        double distanceToTag = Double.MAX_VALUE;
        for (LLResultTypes.FiducialResult fiducialResult : result.getFiducialResults()) {
            Position tagPosition = fiducialResult.getRobotPoseTargetSpace().getPosition().toUnit(DistanceUnit.INCH);
            double distance = Math.hypot(tagPosition.x, Math.hypot(tagPosition.y, tagPosition.z));
            distanceToTag = Math.min(distanceToTag, distance);
        }

        double[] res = new double[2];
        double[] mt1 = result.getStddevMt1();
        double[] mt2 = result.getStddevMt2();

        if (distanceToTag <= 36) res[1] = mt1[5];
        if (distanceToTag <= 132) {
            res[0] = Math.hypot(mt2[0], Math.hypot(mt2[1], mt2[2]));
        }

        return res;
    }

    public Limelight3A getLimelight() {
        return limelight;
    }

    /**
     * Normalizes a given angle to (-pi,pi] radians.
     * @param radians the given angle in radians.
     * @return the normalized angle in radians.
     */
    private static double normalizeAngle(double radians) {
        while (radians > Math.PI) radians -= 2*Math.PI;
        while (radians <= -Math.PI) radians += 2*Math.PI;
        return radians;
    }

    /**
     * Contains the pose and uncertainties for an AprilTag pose estimation.
     */
    public static class PoseEstimation {
        public Pose2d pose;
        public double translationCovariance;
        public double headingCovariance;

        public PoseEstimation(Pose2d pose, double translationCovariance, double headingCovariance) {
            this.pose = pose;
            this.translationCovariance = translationCovariance;
            this.headingCovariance = headingCovariance;
        }
    }

    public enum SampleColor {
        RED,
        YELLOW,
        BLUE,
        BY,
        RY
    }
    public void switchTargetedSampleColor(SampleColor newColor) {
        currentTargetColor = newColor;
    }

    /**
     * Returns the (x, y) offset of the center of the current targeted color from the top left corner
     */
    public Point getSampleOffset() {
        // TODO: IMPLEMENT
        return null;
    }

    /**
     * @return if the detectedSamples list contains any results.
     */
    public boolean detectedAnySamples() {
        if (detectedSamples == null) return false;
        return !detectedSamples.isEmpty();
    }

    /**
     * @param currentPose the robot's current position
     * @return null if there are no samples, otherwise the sample closest to the given position.
     */
    public Pose2d getClosestDetectedSample(Pose2d currentPose) {
        if (detectedSamples == null) return null;
        if (detectedSamples.isEmpty()) return null;
        Pose2d closestPose = detectedSamples.get(0);
        double closestDistance = detectedSamples.get(0).getTranslation().minus(currentPose.getTranslation()).getNorm();
        for (int i = 1; i < detectedSamples.size(); i++) {
            Pose2d samplePose = detectedSamples.get(i);
            double distance = samplePose.getTranslation().minus(currentPose.getTranslation()).getNorm();
            if (distance < closestDistance) {
                closestPose = samplePose;
                closestDistance = distance;
            }
        }
        return closestPose;
    }

}