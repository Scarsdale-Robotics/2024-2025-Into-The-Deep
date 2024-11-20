package org.firstinspires.ftc.teamcode.synchropather.systems.lift;

import com.acmerobotics.dashboard.config.Config;

/**
 * A static class used by Lift Movements to reference important kinematic and algorithmic tuning values.
 */
@Config
public class LiftConstants {

    /**
     *  Max lift velocity in ticks/s.
     */
    public static double MAX_VELOCITY = 2200;
    /**
     *  Max lift acceleration in ticks/s^2.
     */
    public static double MAX_ACCELERATION = 2200;

}
