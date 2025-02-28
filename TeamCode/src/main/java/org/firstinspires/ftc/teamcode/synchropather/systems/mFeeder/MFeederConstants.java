package org.firstinspires.ftc.teamcode.synchropather.systems.mFeeder;

import com.acmerobotics.dashboard.config.Config;

/**
 * A static class used by Magazine Feeder Movements to reference important kinematic and algorithmic tuning values.
 */
@Config
public class MFeederConstants {

    /**
     *  Conversion factor
     */
    public static double TICKS_PER_INCH = 328;

    public static double ticksToInches(double ticks) {
        return ticks / TICKS_PER_INCH;
    }

    /**
     *  Max magazine feeder velocity in inches/s.
     */
    public static double MAX_VELOCITY = 9;

    /**
     *  Max magazine feeder acceleration in inches/s^2.
     */
    public static double MAX_ACCELERATION = 24;

    /**
     *  The magazine feeder's reference position for zero in inches
     */
    public static double ZERO_HOME = 0.25;

    /**
     *  Max capacity of magazine in clips.
     */
    public static int MAX_CAPACITY = 7; //TODO: TUNE

    /**
     * The amount of clips that the magazine intakes.
     */
    public static int RELOAD_CAPACITY = 5;

    /**
     * Width of one clip
     */
    public static double INCHES_PER_CLIP = 1; // TODO: TUNE

}
