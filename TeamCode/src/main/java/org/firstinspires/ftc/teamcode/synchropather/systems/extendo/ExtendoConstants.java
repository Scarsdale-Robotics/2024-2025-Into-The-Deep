package org.firstinspires.ftc.teamcode.synchropather.systems.extendo;

import com.acmerobotics.dashboard.config.Config;

/**
 * A static class used by Extendo Movements to reference important kinematic and algorithmic tuning values.
 */
@Config
public class ExtendoConstants {

    /**
     *  Conversion factor
     */
    public static double TICKS_PER_INCH = 22.211;

    private static double ticksToInches(double ticks) {
        return ticks / TICKS_PER_INCH;
    }

    /**
     *  Max extendo velocity in inches/s.
     */
    public static double MAX_VELOCITY = 105;

    /**
     *  Max extendo acceleration in inches/s^2.
     */
    public static double MAX_ACCELERATION = ticksToInches(2000);

    /**
     *  The distance from the center of the claw to the robot's center of rotation when the extendo is fully retracted and the claw is at pickup position.
     */
    public static double CLAW_OFFSET_DISTANCE = 3.47830709;

    /**
     *  Max extendo extension length in inches.
     */
    public static double MAX_EXTENSION = ticksToInches(633);

}
