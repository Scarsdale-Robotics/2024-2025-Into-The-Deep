package org.firstinspires.ftc.teamcode.opmodes.calibration.ArmTesting;

import org.ejml.simple.SimpleMatrix;

public class Servo3RArmIK {

    private final double armBaseZero = Servo3RArmConstants.armBaseZero;
    private final double armBaseNinety = Servo3RArmConstants.armBaseNinety;

    private final double armFirstJointZero = Servo3RArmConstants.armFirstJointZero;
    private final double armFirstJointNinety = Servo3RArmConstants.armFirstJointNinety;

    private final double armSecondJointZero = Servo3RArmConstants.armSecondJointZero;
    private final double armSecondJointNinety = Servo3RArmConstants.armSecondJointNinety;

    // Measurements in inches
    private final double armFirstSegmentLength = Servo3RArmConstants.armFirstSegmentLength;
    private final double armSecondSegmentLength = Servo3RArmConstants.armSecondSegmentLength;
    private final double maxRho = Servo3RArmConstants.maxRho;
    private final double minRho = Servo3RArmConstants.minRho;

    private double[] cartesian;
    private double[] spherical;
    private double[] servoArmAngles;
    private double[] servoArmPositions;


    public Servo3RArmIK() {}


    /**
     * @param cartesian {x, y, z} inches
     * @return {alphaPosition, betaPosition, gammaPosition} in [-1,1]
     */
    public double[] getServoArmPositions(double[] cartesian) {
        this.cartesian = cartesian;
        this.spherical = cartesianToSpherical(this.cartesian);
        this.servoArmAngles = toServoArmAngles(this.spherical);
        this.servoArmPositions = toServoArmPositions(this.servoArmAngles);
        return this.servoArmPositions;
    }

    /**
     * @param velocity inches/second
     * @return d/dt{alpha, beta, gamma} radians/second
     */
    public double[] getServoAnglesJacobian(double[] velocity) {
        double L1L2 = armFirstSegmentLength*armSecondSegmentLength;
        double x = cartesian[0];
        double y = cartesian[1];
        double z = cartesian[2];
        double vx = velocity[0];
        double vy = velocity[1];
        double vz = velocity[2];
        double rho = spherical[0];
        double rrho = rho*rho;
        double r = Math.hypot(x, y);
        double rr = r*r;

        // Create D(A) jacobian
        double dBetaDRho = 1 / Math.sqrt(4*L1L2 - rrho);
        SimpleMatrix DA = new SimpleMatrix(
                new double[][] {
                        {0,           1,  0},
                        {-dBetaDRho,  0, -1},
                        {2*dBetaDRho, 0,  0}
                }
        );

        // Create D(R) jacobian
        SimpleMatrix DR = new SimpleMatrix(
                new double[][] {
                        {       x/rho,        y/rho,   z/rho},
                        {       -y/rr,         x/rr,       0},
                        {x*z/(r*rrho), y*z/(r*rrho), -r/rrho}
                }
        );

        // Create dr/dt vector
        SimpleMatrix dr_dt = new SimpleMatrix(
                new double[][] {
                        {vx},
                        {vy},
                        {vz}
                }
        );

        // Get resultant jacobian
        SimpleMatrix servoAnglesJacobian = (DA.mult(DR)).mult(dr_dt);
        return new double[]{
                servoAnglesJacobian.get(0,0),
                servoAnglesJacobian.get(1,0),
                servoAnglesJacobian.get(2,0)
        };
    }

    /**
     * @param servoAnglesJacobian d/dt{alpha, beta, gamma} radians/second
     * @return d/dt{alphaPosition, betaPosition, gammaPosition}
     */
    public double[] getServoPositionsJacobian(double[] servoAnglesJacobian) {
        double dAlphaDt = servoAnglesJacobian[0];
        double dBetaDt = servoAnglesJacobian[1];
        double dGammaDt = servoAnglesJacobian[2];
        return new double[] {
                toServoPositionScaled(dAlphaDt, armBaseZero, armBaseNinety),
                toServoPositionScaled(dBetaDt, armFirstJointZero, armFirstJointNinety),
                toServoPositionScaled(dGammaDt, armSecondJointZero, armSecondJointNinety)
        };
    }

    /**
     * @param cartesian {x, y, z} inches
     * @return {rho, theta, phi} inches, radians
     */
    private double[] cartesianToSpherical(double[] cartesian) {
        double x = cartesian[0];
        double y = cartesian[1];
        double z = cartesian[2];

        // Convert to spherical
        double rho = Math.sqrt(x*x + y*y + z*z);
        double theta = Math.atan2(y, x);
        double phi = Math.atan2(Math.hypot(x,y), z);
        return new double[]{rho, theta, phi};
    }

    /**
     * @param spherical {rho, theta, phi} inches, radians
     * @return {alpha, beta, gamma} radians
     */
    private double[] toServoArmAngles(double[] spherical) {
        double rho = clamp(spherical[0], minRho, maxRho);
        double theta = spherical[1];
        double phi = spherical[2];

        // Convert to arm angles
        double gamma = Math.acos(-(rho*rho)/(2*armFirstSegmentLength*armSecondSegmentLength) + 1);
        double beta = Math.PI - phi - gamma/2;
        double alpha = theta;
        return new double[]{alpha, beta, gamma};
    }

    /**
     * @param servoArmAngles {alpha, beta, gamma} radians
     * @return {alphaPosition, betaPosition, gammaPosition} in [-1,1]
     */
    private double[] toServoArmPositions(double[] servoArmAngles) {
        double alpha = servoArmAngles[0];
        double beta = servoArmAngles[1];
        double gamma = servoArmAngles[2];

        // Convert to arm positions
        double alphaPosition = toServoArmPosition(alpha, armBaseZero, armBaseNinety);
        double betaPosition = toServoArmPosition(beta, armFirstJointZero, armFirstJointNinety);
        double gammaPosition = toServoArmPosition(gamma, armSecondJointZero, armSecondJointNinety);
        return new double[]{alphaPosition, betaPosition, gammaPosition};
    }

    /**
     * Performs linear interpolation.
     * @param servoArmAngle radians
     * @param zeroPosition 0 degrees servo position
     * @param ninetyPosition 90 degrees servo position
     * @return desired servo position
     */
    private double toServoArmPosition(double servoArmAngle, double zeroPosition, double ninetyPosition) {
        // Linear interpolation
        double servoArmPosition = zeroPosition + toServoPositionScaled(servoArmAngle, zeroPosition, ninetyPosition);
        servoArmPosition = clamp(servoArmPosition, -1, 1);
        return servoArmPosition;
    }

    private double toServoPositionScaled(double servoArmAngle, double zeroPosition, double ninetyPosition) {
        return servoArmAngle*(ninetyPosition - zeroPosition)/(Math.PI/2 - 0);
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
