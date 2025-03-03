package org.firstinspires.ftc.teamcode.synchropather.systems.hArm;

import com.acmerobotics.dashboard.config.Config;

/**
 * A static class used by horizontal arm Movements to reference important kinematic and algorithmic tuning values.
 */
@Config
public class HArmConstants {

    // Left servo arm homing positions
    public static double armLeftZeroPosition = 0.85;
    public static double armLeftPiPosition = 0.15;
    public static double armLeftDownPosition = 1.04;

    // Right servo arm homing positions
    public static double armRightZeroPosition = 0.16;
    public static double armRightPiPosition = 0.86;

    // For transfer
    public static double armTransferPosition = 0.46;

    /**
     *  Max elbow velocity in servo/sec.
     */
    public static double MAX_VELOCITY = positionToScaled(2);

    /**
     *  Max elbow acceleration in servo/sec^2.
     */
    public static double MAX_ACCELERATION = positionToScaled(4);

    private static double positionToScaled(double position) {
        return position / Math.abs(armLeftPiPosition - armLeftZeroPosition);
    }

}
