package org.firstinspires.ftc.teamcode.synchropather.systems.magazine;

import com.acmerobotics.dashboard.config.Config;

/**
 * A static class used by Magazine Movements to reference important kinematic and algorithmic tuning values.
 */
@Config
public class MagazineConstants {

    /**
     *  Conversion factor
     */
    public static double TICKS_PER_INCH = 50;//TODO: TUNE

    public static double ticksToInches(double ticks) {
        return ticks / TICKS_PER_INCH;
    }

    /**
     *  Max magazine velocity in inches/s.
     */
    public static double MAX_VELOCITY = ticksToInches(2500);

    /**
     *  Max magazine acceleration in inches/s^2.
     */
    public static double MAX_ACCELERATION = ticksToInches(8000);

    /**
     *  The magazine's reference position for zero in inches
     */
    public static double ZERO_HOME = 0;

    /**
     *  Max magazine position in inches.
     */
    public static double MAX_POSITION = 9; //TODO: TUNE

}
