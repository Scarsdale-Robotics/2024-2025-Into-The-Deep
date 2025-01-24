package org.firstinspires.ftc.teamcode.opmodes.calibration.ArmTesting;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Servo3RArmController {

    /**
     * Feedforward control coefficient
     */
    public static double kV = 0.1;

    private final Servo servoArmBase;
    private final Servo servoArmFirstJoint;
    private final Servo servoArmSecondJoint;

    public Servo3RArmController(Servo servoArmBase, Servo servoArmFirstJoint, Servo servoArmSecondJoint) {
        this.servoArmBase = servoArmBase;
        this.servoArmFirstJoint = servoArmFirstJoint;
        this.servoArmSecondJoint = servoArmSecondJoint;
    }

    /**
     * @param servoArmPositions {alphaPosition, betaPosition, gammaPosition} in [-1,1]
     */
    public void setPosition(double[] servoArmPositions) {
        double alphaPosition = servoArmPositions[0];
        double betaPosition = servoArmPositions[1];
        double gammaPosition = servoArmPositions[2];
        servoArmBase.setPosition(alphaPosition);
        servoArmFirstJoint.setPosition(betaPosition);
        servoArmSecondJoint.setPosition(gammaPosition);
    }

    /**
     * @param servoArmPositions {alphaPosition, betaPosition, gammaPosition} in [-1,1]
     * @param servoPositionsJacobian d/dt{alphaPosition, betaPosition, gammaPosition}
     */
    public void setPositionAndFeedforward(double[] servoArmPositions, double[] servoPositionsJacobian) {
        // Get positions and velocities
        double alphaPosition = servoArmPositions[0];
        double betaPosition = servoArmPositions[1];
        double gammaPosition = servoArmPositions[2];
        double dAlphaPositionDt = servoPositionsJacobian[0];
        double dBetaPositionDt = servoPositionsJacobian[0];
        double dGammaPositionDt = servoPositionsJacobian[0];

        // Get control outputs
        double ut_alphaPosition = clamp(alphaPosition + kV*dAlphaPositionDt, -1, 1);
        double ut_betaPosition = clamp(betaPosition + kV*dBetaPositionDt, -1, 1);
        double ut_gammaPosition = clamp(gammaPosition + kV*dGammaPositionDt, -1, 1);
        servoArmBase.setPosition(ut_alphaPosition);
        servoArmFirstJoint.setPosition(ut_betaPosition);
        servoArmSecondJoint.setPosition(ut_gammaPosition);
    }

    /**
     * Clamps the given value between the given bounds.
     * @param x value
     * @param lower bound
     * @param upper bound
     * @return Math.max(lower, Math.min(upper, x))
     */
    private double clamp(double x, double lower, double upper) {
        return Math.max(lower, Math.min(upper, x));
    }
}
