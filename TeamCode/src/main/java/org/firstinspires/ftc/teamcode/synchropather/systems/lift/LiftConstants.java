package org.firstinspires.ftc.teamcode.synchropather.systems.lift;

import com.acmerobotics.dashboard.config.Config;

/**
 * A static class used by Lift Movements to reference important kinematic and algorithmic tuning values.
 */
@Config
public class LiftConstants {


    public static double specMakerPosition = 0.0;
    public static double transferPosition = 9.45;
    public static double depositPosition = 16;  // TODO: TUNE

    /**
     *  Conversion factor
     */
    public static double TICKS_PER_INCH = 42.202;

    private static double ticksToInches(double ticks) {
        return ticks / TICKS_PER_INCH;
    }

    /**
     *  Max lift velocity in inches/s.
     */
    public static double MAX_MOTOR_VELOCITY = 51;

    public static double MAX_PATHING_VELOCITY = 40;

    /**
     *  Max lift acceleration in inches/s^2.
     */
    public static double MAX_ACCELERATION = ticksToInches(6000);

    /**
     *  Max lift deceleration in inches/s^2.
     */
    public static double MAX_DECELERATION = ticksToInches(3000);

    /**
     * Maximum lift height in inches.
     */
    public static double MAX_HEIGHT = 27.25;

}
