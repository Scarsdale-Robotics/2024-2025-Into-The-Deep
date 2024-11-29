package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.Function;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.RobotSystem;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Point;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Date;
import java.util.List;
import java.util.Objects;
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

    private RobotSystem robot;

    //////////////////
    // INIT METHODS //
    //////////////////

    public CVSubsystem(Limelight3A limelight, double initialHeading, boolean isRedTeam, Telemetry telemetry, RobotSystem robot) {
        this.limelight = limelight;
        this.limelight.updateRobotOrientation(Math.toDegrees(initialHeading));
        this.telemetry = telemetry;

        this.robot = robot;
  
        // Switch to AprilTag
        this.limelight.pipelineSwitch(2);
        this.limelight.start();

        currentTargetColor = isRedTeam ? SampleColor.RY : SampleColor.BY;
    }



    //////////////////
    // MAIN METHODS //
    //////////////////


    private final Double DNE = -1.;
    ArrayList<Double> ttsPrevTX, ttsPrevTY, ttsPrevDT;
    private void resetTTSVars() {
        ttsPrevTX = new ArrayList<>();
        ttsPrevTY = new ArrayList<>();
        ttsPrevDT = new ArrayList<>();
        ttsPrevTX.addAll(Collections.nCopies(5, DNE));
        ttsPrevTY.addAll(Collections.nCopies(5, DNE));
        ttsPrevDT.addAll(Collections.nCopies(5, DNE));
    }

    private double derivative(ArrayList<Double> prevpos, ArrayList<Double> prevtime) {
        return (
                -prevpos.get(0) + 8*prevpos.get(1) - 8*prevpos.get(3) + prevpos.get(4)
        ) / (
                12*(prevtime.get(4) - prevtime.get(0))
        );
    }

    public enum Color { RED, YELLOW, BLUE }

    private double ttsLastTime = new Date().getTime();
    public boolean tickTowardsSample(boolean active, double maxSpeed, Set<Color> colors) {
        double KPX = 0.03, KDX = 0.00;
        double KPY = 0.60, KDY = 0.30;

        Function<Double, Double> clamp = (v) -> Math.max(Math.min(v, maxSpeed), -maxSpeed);
        Runnable pause = () -> {
            robot.drive.driveRobotCentricPowers(0, 0, 0);
            telemetry.addData("NOTE","NO RESULT");
//            telemetry.update();
        };

        if (!active) {
            resetTTSVars();
            return false;
        }

        telemetry.addData("state","running");
        LLResult result = limelight.getLatestResult();

        if (result == null || result.isValid()) {
            telemetry.addData("PLoc", "0");
            return true;
        }

        List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();
        List<LLResultTypes.DetectorResult> selected = new ArrayList<>();

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

            telemetry.addData("CL", className);

            if (colors.contains(detectedColor)) selected.add(detection);
        }

        if (selected.isEmpty()) {
            telemetry.addData("PLoc", "1");
            telemetry.addData("DS", detections.size());
            pause.run();
            return true;
        }

        double closestDetectionDist = Double.MAX_VALUE;
        LLResultTypes.DetectorResult closestDetection = detections.get(0);
        for (LLResultTypes.DetectorResult detection : selected) {
            double tx = detection.getTargetXDegrees(); // Where it is (left-right)
            double ty = detection.getTargetYDegrees(); // Where it is (up-down)
            double d = Math.sqrt(Math.pow(tx, 2) + Math.pow(ty, 2));  // could have just done tx*tx + ty*ty but this way of writing imo shows the formula much clearer to warrant the extra text
            if (d < closestDetectionDist) {
                closestDetectionDist = d;
                closestDetection = detection;
            }
        }
//        telemetry.addData("cdd", closestDetectionDist);

        double tx = closestDetection.getTargetXDegrees(); // Where it is (left-right)
        double ty = closestDetection.getTargetYDegrees(); // Where it is (up-down)

        double t = new Date().getTime();
        double dt = (t - ttsLastTime) / 1000.;
        ttsLastTime = t;
        ttsPrevDT.remove(0); ttsPrevDT.add(dt);
        ttsPrevTX.remove(0); ttsPrevTX.add(dt);

        boolean derv_vld = !Objects.equals(ttsPrevTX.get(0), DNE);//what does this do?

        double dx = derv_vld ? derivative(ttsPrevTX, ttsPrevDT) : 0;//what does this do?
        double dy = derv_vld ? derivative(ttsPrevTY, ttsPrevDT) : 0;//what does this do?

        double u_tx = KPX*tx + KDX*dx; clamp.apply(u_tx);
        double u_ty = KPY*ty + KDY*dy; clamp.apply(u_ty);

        robot.drive.driveRobotCentricPowers(u_tx,-u_ty,0);
        telemetry.update();

        return true;
    }

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
        double tagSkew = -69;
        for (LLResultTypes.FiducialResult fiducialResult : result.getFiducialResults()) {
            Position tagPosition = fiducialResult.getRobotPoseTargetSpace().getPosition().toUnit(DistanceUnit.INCH);
            double distance = Math.hypot(tagPosition.x, Math.hypot(tagPosition.y, tagPosition.z));
            distanceToTag = Math.min(distanceToTag, distance);
            tagYaw = fiducialResult.getRobotPoseTargetSpace().getOrientation().getPitch();
            tagSkew = fiducialResult.getSkew();
        }
        telemetry.addData("distanceToTag", distanceToTag);
        telemetry.addData("tagYaw", tagYaw);
        telemetry.addData("tagSkew", tagSkew);
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
        return (radians + Math.PI) % (2*Math.PI) - Math.PI;
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

}