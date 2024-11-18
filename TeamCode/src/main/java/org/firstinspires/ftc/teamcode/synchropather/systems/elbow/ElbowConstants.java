package org.firstinspires.ftc.teamcode.synchropather.systems.elbow;

import com.acmerobotics.dashboard.config.Config;

/**
 * A static class used by Elbow Movements to reference important kinematic and algorithmic tuning values.
 */
@Config
public class ElbowConstants {

    /**
     *  Max elbow velocity in servovalue/s.
     */
    public static double MAX_VELOCITY = 1;
    /**
     *  Max elbow acceleration in servovalue/s^2.
     */
    public static double MAX_ACCELERATION = 1;

}
