package org.firstinspires.ftc.teamcode.synchropather.systems.lift;

import com.acmerobotics.dashboard.config.Config;

/**
 * A static class used by Lift Movements to reference important kinematic and algorithmic tuning values.
 */
@Config
public class LiftConstants {

    /**
     *  Max lift velocity in inches/s.
     */
    public static double MAX_VELOCITY = ticksToInches(2500);

    /**
     *  Max lift acceleration in inches/s^2.
     */
    public static double MAX_ACCELERATION = ticksToInches(4000);

    /**
     *  Conversion factor
     */
    public static double TICKS_PER_INCH = 25;  // TODO: TUNE

    private static double ticksToInches(double ticks) {
        return ticks / TICKS_PER_INCH;
    }

}
