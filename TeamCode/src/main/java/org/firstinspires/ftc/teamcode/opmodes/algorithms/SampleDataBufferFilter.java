package org.firstinspires.ftc.teamcode.opmodes.algorithms;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.LocalizationSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.subsystemclasses.LinearSlidesSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.subsystemclasses.OverheadCameraSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.systems.extendo.ExtendoState;

import java.util.ArrayList;
import java.util.Collections;

@Config
public class SampleDataBufferFilter {

    private final LinearSlidesSubsystem linearSlides;
    private final LocalizationSubsystem localization;

    // For data buffering to account for the camera's lag
    private final ElapsedTime runtime;
    private final double timeBuffer;
    private final ArrayList<double[]> bufferedExtendoPositions;  // [{position, timestamp}, ...]
    private final ArrayList<Pose2d> bufferedBotPoses;
    private ExtendoState lastBufferedExtendoPosition;
    private Pose2d lastBufferedBotPose;

    // For filtering incoming cv sample data
    private final int filterLength;
    private final ArrayList<Pose2d> samplePoses; // {x_sample, y_sample, theta_sample}

    public SampleDataBufferFilter(LinearSlidesSubsystem linearSlides, LocalizationSubsystem localization, double timeBuffer, int filterLength) {
        this.linearSlides = linearSlides;
        this.localization = localization;

        // init buffer
        runtime = new ElapsedTime(0);
        runtime.reset();
        this.timeBuffer = timeBuffer;
        bufferedExtendoPositions = new ArrayList<>();
        bufferedBotPoses = new ArrayList<>();
        lastBufferedExtendoPosition = null;
        lastBufferedBotPose = null;

        // init filter
        this.filterLength = filterLength;
        samplePoses = new ArrayList<>();
    }

    /**
     * Initializes with timeBuffer=0.045, and filterLength=5.
     */
    public SampleDataBufferFilter(LinearSlidesSubsystem linearSlides, LocalizationSubsystem localization) {
        this(linearSlides, localization, 0.045, 5);
    }

    /**
     * Updates the filter with most recent sample data
     * @param samplePosition
     */
    public void updateFilterData(double[] samplePosition) {
        if (samplePosition==null || lastBufferedExtendoPosition==null || lastBufferedBotPose==null) return;

        //// Append current data to end of filter
        // calculate sample position
        ExtendoState extendoPosition = lastBufferedExtendoPosition;
        Pose2d botPose = lastBufferedBotPose;

        // unpack bot pose & extendo state
        double x_bot = botPose.getX();
        double y_bot = botPose.getY();
        double heading_bot = botPose.getHeading();
        double x_extendo = extendoPosition.getLength();

        // get cv readings
        double x_sample_cam = samplePosition[0];
        double y_sample_cam = samplePosition[1];
        double theta_sample_cam = samplePosition[2];

        // convert to robot frame
        double x_sample_bot = OverheadCameraSubsystem.CAMERA_OFFSET[0] + x_extendo + x_sample_cam;
        double y_sample_bot = OverheadCameraSubsystem.CAMERA_OFFSET[1] + y_sample_cam;
        double theta_sample_bot = Math.atan2(y_sample_bot, x_sample_bot);
        double distance_sample_bot = Math.hypot(x_sample_bot, y_sample_bot);
        double bearing = heading_bot + theta_sample_bot;

        // find real sample coordinates
        double x_sample = x_bot + distance_sample_bot * Math.cos(bearing);
        double y_sample = y_bot + distance_sample_bot * Math.sin(bearing);
        double theta_sample = (theta_sample_cam + heading_bot) - Math.PI/2;

        samplePoses.add(new Pose2d(
                x_sample,
                y_sample,
                new Rotation2d(theta_sample)
        ));


        //// Remove data outside filter
        while (samplePoses.size()>filterLength) samplePoses.remove(0);

    }

    public void clearFilterData() {
        samplePoses.clear();
    }

    /**
     * @return if samplePoses.size()==filterLength.
     */
    public boolean isFilterFull() {
        return samplePoses.size()==filterLength;
    }

    /**
     * @return the average sample position (relative to the field).
     */
    public Pose2d getFilteredSamplePosition() {
        if (!isFilterFull()) return null;

        ArrayList<Double> x_sample_sorted = new ArrayList<>();
        ArrayList<Double> y_sample_sorted = new ArrayList<>();
        double[] theta_sample_vector_sum = new double[]{0,0};
        for (Pose2d samplePose : samplePoses) {
            x_sample_sorted.add(samplePose.getX());
            y_sample_sorted.add(samplePose.getY());
            theta_sample_vector_sum[0] += Math.cos(samplePose.getHeading());
            theta_sample_vector_sum[1] += Math.sin(samplePose.getHeading());
        }
        Collections.sort(x_sample_sorted);
        Collections.sort(y_sample_sorted);

        // Use median for x and y
        // Use mean for angle because median would be very difficult due to wrapping
        double x_sample;
        double y_sample;
        double theta_sample;

        // get x_sample, y_sample
        double x1 = x_sample_sorted.get(filterLength/2);
        double y1 = y_sample_sorted.get(filterLength/2);
        if (filterLength%2 == 0) {
            double x2 = x_sample_sorted.get(filterLength/2 - 1);
            double y2 = y_sample_sorted.get(filterLength/2 - 1);
            x_sample = (x1+x2)/2.0;
            y_sample = (y1+y2)/2.0;
        } else {
            x_sample = x1;
            y_sample = y1;
        }

        // get theta_sample
        if (Math.hypot(theta_sample_vector_sum[0],theta_sample_vector_sum[1])==0) {
            theta_sample = samplePoses.get(0).getHeading();
        } else {
            theta_sample = Math.atan2(theta_sample_vector_sum[1],theta_sample_vector_sum[0]);
        }

        return new Pose2d(x_sample, y_sample, new Rotation2d(theta_sample));
    }



    /**
     * Add the most recent measurements to the buffer and clear old measurements.
     */
    public void updateBufferData() {
        double currentTime = runtime.seconds();

        // extendo
        double currentExtendoPosition = linearSlides.getExtendoPosition();
        bufferedExtendoPositions.add(new double[]{
                currentExtendoPosition,
                currentTime
        });

        // botpose
        Pose2d currentBotPose = localization.getPose();
        bufferedBotPoses.add(
                currentBotPose
        );

        // clear data outside buffer
        while (currentTime - bufferedExtendoPositions.get(0)[1] > timeBuffer) {
            bufferedExtendoPositions.remove(0);
            bufferedBotPoses.remove(0);
        }

        // store latest data
        lastBufferedExtendoPosition = new ExtendoState(bufferedExtendoPositions.get(0)[0]);
        lastBufferedBotPose = bufferedBotPoses.get(0);
    }

}
