package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.Consumer;
import org.firstinspires.ftc.robotcore.external.Function;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.teamcode.RobotSystem;
import org.firstinspires.ftc.teamcode.cvpipelines.processors.VanillaPer_ocessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Date;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Set;
import java.util.concurrent.Callable;


public class CVSubsystem extends SubsystemBase {

    /**
     * The camera position relative to the drivetrain's (facing rightwards) center of rotation, in inches and radians.
     * Heading is the cartesian angle towards which the camera points, with the drivetrain facing rightwards.
     */
    private static final Pose2d CAMERA_POSITION = new Pose2d(0, 0, new Rotation2d(0));
    private static final Map<Integer, Pose2d> APRIL_TAG_LOCATIONS = new HashMap<>();

    private final WebcamName cameraName;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private VanillaPer_ocessor VaPe;

    private final Telemetry telemetry;

    private final RobotSystem robot;

    private Limelight3A limelight;


    //////////////////
    // INIT METHODS //
    //////////////////

    public CVSubsystem(WebcamName cameraName, Telemetry telemetry, RobotSystem robot) {
        this.telemetry = telemetry;

        this.cameraName = cameraName;

        this.robot = robot;

        this.limelight = robot.hardwareRobot.limelight;
        limelight.start();

        resetTTSVars();

        // Construct AprilTag locations map.
        initAprilTagLocations();

        // Create AprilTagProcessor and VisionPortal.
        initVisionPortal();
    }

    /**
     * Store the positions of the AprilTags on the field.
     */
    private void initAprilTagLocations() {
        // Poses are in the format (X, Y, H), where:
        //  - X and Y are in inches according to https://ftc-docs.firstinspires.org/en/latest/game_specific_resources/field_coordinate_system/field-coordinate-system.html#reference-frame.
        //  - H is the cartesian angle that the left side of the AprilTag is pointing towards.

        APRIL_TAG_LOCATIONS.put(11, new Pose2d(-3*24,  2*24, new Rotation2d(-Math.PI/2)));
        APRIL_TAG_LOCATIONS.put(12, new Pose2d( 0   ,  3*24, new Rotation2d(-Math.PI)));
        APRIL_TAG_LOCATIONS.put(13, new Pose2d( 3*24,  2*24, new Rotation2d(Math.PI/2)));
        APRIL_TAG_LOCATIONS.put(14, new Pose2d( 3*24, -2*24, new Rotation2d(Math.PI/2)));
        APRIL_TAG_LOCATIONS.put(15, new Pose2d( 0   , -3*24, new Rotation2d(0)));
        APRIL_TAG_LOCATIONS.put(16, new Pose2d(-3*24, -2*24, new Rotation2d(-Math.PI/2)));
    }

    /**
     * Creates a VisionPortal with an AprilTag processor.
     */
    private void initVisionPortal() {
        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawTagOutline(true)
                .build();
//        VaPe = new VanillaPer_ocessor();


        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(cameraName);
        builder.setCameraResolution(new Size(640, 480));
//        builder.addProcessors(VaPe, aprilTag);
        builder.addProcessors(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();
        visionPortal.setProcessorEnabled(VaPe, true);
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
            telemetry.update();
        };

        if (!active) {
            resetTTSVars();
            return false;
        }

        telemetry.addData("state","running");
        LLResult result = limelight.getLatestResult();

        if (result == null || result.isValid()) {
            pause.run();
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

            if (colors.contains(detectedColor)) selected.add(detection);
        }

        if (selected.isEmpty()) {
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

        boolean derv_vld = !Objects.equals(ttsPrevTX.get(0), DNE);

        double dx = derv_vld ? derivative(ttsPrevTX, ttsPrevDT) : 0;
        double dy = derv_vld ? derivative(ttsPrevTY, ttsPrevDT) : 0;

        double u_tx = KPX*tx + KDX*dx; clamp.apply(u_tx);
        double u_ty = KPY*ty + KDY*dy; clamp.apply(u_ty);

        robot.drive.driveRobotCentricPowers(u_tx,-u_ty,0);
        telemetry.update();

        return true;
    }

    /**
     * Logs the poses of the detected AprilTags relative to the robot.
     */
    public void telemeterRelativeAprilTags() {
        for (AprilTagDetection detection : aprilTag.getDetections()) {
            telemetry.addData(
                    String.format("Tag %s's Rel. Pose", detection.id),
                    String.format("(X:%s, Y:%s)", detection.rawPose.x, detection.rawPose.y)
            );
        }
        telemetry.update();
    }

    /**
     * Logs the robot poses calculated using the detected AprilTags relative to the <a href="https://ftc-docs.firstinspires.org/en/latest/game_specific_resources/field_coordinate_system/field-coordinate-system.html#reference-frame">field reference frame</a>.
     */
    public void telemeterAbsoluteAprilTags() {
        for (AprilTagDetection detection : aprilTag.getDetections()) {
            if (APRIL_TAG_LOCATIONS.get(detection.id) == null) continue;

            Pose2d absolutePose = getPoseEstimation(detection).pose;
            telemetry.addData(
                    String.format("Real Pose from Tag %s", detection.id),
                    String.format("(X:%s, Y:%s, H:%s)", absolutePose.getX(), absolutePose.getY(), absolutePose.getHeading())
            );
        }
        telemetry.update();
    }

    /**
     * @param detection the detected AprilTag.
     * @return the robot pose estimation calculated using the given AprilTag.
     */
    private PoseEstimation getPoseEstimation(AprilTagDetection detection) {
        Orientation rot = Orientation.getOrientation(detection.rawPose.R, AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        Pose2d tagPose = Objects.requireNonNull(APRIL_TAG_LOCATIONS.get(detection.id));

        // Setup.
        double bearing = Math.atan2(detection.rawPose.y, detection.rawPose.x);
        double yaw = rot.thirdAngle;
        double range = Math.hypot(detection.rawPose.x, detection.rawPose.y);
        double tagHeading = tagPose.getHeading();

        // Translate to camera pose.
        double theta_real = tagHeading + bearing - yaw;
        double x_camera = tagPose.getX() + range * Math.cos(theta_real);
        double y_camera = tagPose.getY() + range * Math.sin(theta_real);
        double h_camera = tagHeading - yaw - Math.PI/2;

        // Translate to robot pose.
        double camera_rel_x = CAMERA_POSITION.getX();
        double camera_rel_y = CAMERA_POSITION.getY();
        double camera_rel_h = CAMERA_POSITION.getHeading();
        double camera_rotation = h_camera - camera_rel_h;

        double dx = camera_rel_x*Math.cos(camera_rotation) - camera_rel_y*Math.sin(camera_rotation);
        double dy = camera_rel_x*Math.sin(camera_rotation) + camera_rel_y*Math.cos(camera_rotation);
        double dh = -camera_rel_h;

        double x_real = x_camera - dx;
        double y_real = y_camera - dy;
        double h_real = normalizeAngle(h_camera + dh);

        Pose2d estimation = new Pose2d(x_real, y_real, new Rotation2d(h_real));

        // Calculate Uncertainty (proportional to range).
        //TODO: TUNE
        double kC = 0.1;

        double translationCovariance = kC*range;
        translationCovariance *= translationCovariance;
        double headingCovariance = kC*range * 180/Math.PI;
        headingCovariance *= headingCovariance;

        return new PoseEstimation(estimation, translationCovariance, headingCovariance);
    }

    /**
     * Chooses the AprilTag pose estimation with the least uncertainty.
     */
    public PoseEstimation getBestPoseEstimation() {
        // Initial condition.
        PoseEstimation bestTranslationEstimation = null;
        PoseEstimation bestHeadingEstimation = null;

        // Loop through detections.
        for (AprilTagDetection detection : aprilTag.getDetections()) {
            if (APRIL_TAG_LOCATIONS.get(detection.id) == null) continue;
            // Independently process translation and heading.
            PoseEstimation estimation = getPoseEstimation(detection);
            if (bestTranslationEstimation==null || estimation.translationCovariance<bestTranslationEstimation.translationCovariance) {
                bestTranslationEstimation = estimation;
            }
            if (bestHeadingEstimation==null || estimation.headingCovariance<bestHeadingEstimation.headingCovariance) {
                bestHeadingEstimation = estimation;
            }
        }
        if (bestTranslationEstimation==null) return null;

        // Splice translation and heading estimates.
        return new PoseEstimation(
                new Pose2d(bestTranslationEstimation.pose.getTranslation(), bestHeadingEstimation.pose.getRotation()),
                bestTranslationEstimation.translationCovariance,
                bestHeadingEstimation.headingCovariance
        );
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

}