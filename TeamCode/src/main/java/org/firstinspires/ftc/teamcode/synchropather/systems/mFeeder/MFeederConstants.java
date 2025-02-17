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
    public static double TICKS_PER_INCH = 50;//TODO: TUNE

    public static double ticksToInches(double ticks) {
        return ticks / TICKS_PER_INCH;
    }

    /**
     *  Max magazine feeder velocity in inches/s.
     */
    public static double MAX_VELOCITY = ticksToInches(2500);

    /**
     *  Max magazine feeder acceleration in inches/s^2.
     */
    public static double MAX_ACCELERATION = ticksToInches(8000);

    /**
     *  The magazine feeder's reference position for zero in inches
     */
    public static double ZERO_HOME = 0;

    /**
     *  Max magazine feeder position in inches.
     */
    public static double MAX_POSITION = 7; //TODO: TUNE

}
