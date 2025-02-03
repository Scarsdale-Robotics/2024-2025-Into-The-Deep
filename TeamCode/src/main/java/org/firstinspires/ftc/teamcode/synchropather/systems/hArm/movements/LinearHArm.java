package org.firstinspires.ftc.teamcode.synchropather.systems.hArm.movements;

import org.firstinspires.ftc.teamcode.synchropather.systems.MovementType;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.TimeSpan;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.motion_profiles.SymmetricMotionProfile1D;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.Movement;
import org.firstinspires.ftc.teamcode.synchropather.systems.hArm.HArmConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.hArm.HArmState;

/**
 * Linear horizontal arm Movement.
 */
public class LinearHArm extends Movement {
    private double distance, minDuration;
    private HArmState start, end;
    private SymmetricMotionProfile1D calculator;

    public LinearHArm(TimeSpan timeSpan, HArmState start, HArmState end) {
        super(timeSpan, MovementType.HORIZONTAL_ARM);
        this.start = start;
        this.end = end;
        init(false, 0);
    }

    /**
     * Creates a new LinearHArm object with a given start and end states at the given startTime.
     * @param startTime
     * @param start
     * @param end
     */
    public LinearHArm(double startTime, HArmState start, HArmState end) {
        super(MovementType.HORIZONTAL_ARM);
        this.start = start;
        this.end = end;
        init(true, startTime);
    }

    @Override
    public double getMinDuration() {
        return minDuration;
    }

    /**
     * @return the indicated HArmState.
     */
    @Override
    public HArmState getState(double elapsedTime) {
        double t = distance!=0 ? calculator.getDisplacement(elapsedTime) / distance : 0;

        double q0 = 1 - t;
        double q1 = t;

        // linear interpolation
        return start.times(q0).plus(end.times(q1));
    }

    /**
     * @return the indicated velocity HArmState.
     */
    @Override
    public HArmState getVelocity(double elapsedTime) {
        double sign = end.minus(start).sign();
        double speed = calculator.getVelocity(elapsedTime);

        // scaled velocity vector
        return new HArmState(sign * speed);
    }

    /**
     * @return the indicated acceleration HArmState.
     */
    @Override
    public HArmState getAcceleration(double elapsedTime) {
        double sign = end.minus(start).sign();
        double speed = calculator.getAcceleration(elapsedTime);

        // scaled acceleration vector
        return new HArmState(sign * speed);
    }

    /**
     * @return the HArmState of this Movement at the start time.
     */
    @Override
    public HArmState getStartState() {
        return start;
    }

    /**
     * @return the HArmState of this Movement at the end time.
     */
    @Override
    public HArmState getEndState() {
        return end;
    }

    /**
     * @return "LinearHArm"
     */
    @Override
    public String getDisplayName() {
        return "LinearHArm";
    }

    /**
     * Calculates total time.
     */
    private void init(boolean startTimeConstructor, double startTime) {
        distance = end.minus(start).abs();

        double MAV = HArmConstants.MAX_VELOCITY;
        double MAA = HArmConstants.MAX_ACCELERATION;

        if (startTimeConstructor) {
            minDuration = SymmetricMotionProfile1D.findMinDuration(distance, MAV, MAA);
            timeSpan = new TimeSpan(startTime, startTime + minDuration);
        }

        // create calculator object
        calculator = new SymmetricMotionProfile1D(distance, timeSpan, MAV, MAA);

        minDuration = calculator.getMinDuration();
    }
}
