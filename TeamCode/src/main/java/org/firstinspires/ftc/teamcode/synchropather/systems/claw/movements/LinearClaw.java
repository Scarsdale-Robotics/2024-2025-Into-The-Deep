package org.firstinspires.ftc.teamcode.synchropather.systems.claw.movements;

import org.firstinspires.ftc.teamcode.synchropather.systems.MovementType;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.TimeSpan;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.calculators.StretchedDisplacementCalculator;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.Movement;
import org.firstinspires.ftc.teamcode.synchropather.systems.claw.ClawConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.claw.ClawState;

public class LinearClaw extends Movement {
    private double distance, minDuration;
    private ClawState start, end;
    private StretchedDisplacementCalculator calculator;

    public LinearClaw(TimeSpan timeSpan, ClawState start, ClawState end) {
        super(timeSpan, MovementType.CLAW);
        this.start = start;
        this.end = end;
        init(false, 0);
    }
    /**
     * Creates a new LinearRotation object with a given start and end RotationState at the given startTime.
     * @param startTime
     * @param start
     * @param end
     */
    public LinearClaw(double startTime, ClawState start, ClawState end) {
        super(MovementType.CLAW);
        this.start = start;
        this.end = end;
        init(true, startTime);
    }

    @Override
    public double getMinDuration() {
        return minDuration;
    }
    /**
     * @return the indicated ClawState.
     */
    @Override
    public ClawState getState(double elapsedTime) {
        double t = distance!=0 ? calculator.getDisplacement(elapsedTime) / distance : 0;

        double q0 = 1 - t;
        double q1 = t;

        // linear interpolation
        return start.times(q0).plus(end.times(q1));
    }
    /**
     * @return the indicated velocity ClawState.
     */
    @Override
    public ClawState getVelocity(double elapsedTime) {
        double sign = end.minus(start).sign();
        double speed = calculator.getVelocity(elapsedTime);

        // scaled velocity vector
        return new ClawState(sign * speed);
    }
    /**
     * @return the indicated acceleration ClawState.
     */
    @Override
    public ClawState getAcceleration(double elapsedTime) {
        double sign = end.minus(start).sign();
        double speed = calculator.getAcceleration(elapsedTime);

        // scaled acceleration vector
        return new ClawState(sign * speed);
    }
    /**
     * @return the ClawState of this Movement at the start time.
     */
    @Override
    public ClawState getStartState() {
        return start;
    }
    /**
     * @return the ClawState of this Movement at the end time.
     */
    @Override
    public ClawState getEndState() {
        return end;
    }

    /**
     * @return "LinearClaw"
     */
    @Override
    public String getDisplayName() {
        return "LinearClaw";
    }

    /**
     * Calculates total time.
     */
    private void init(boolean startTimeConstructor, double startTime) {
        distance = end.minus(start).abs();

        double MAV = ClawConstants.MAX_VELOCITY;
        double MAA = ClawConstants.MAX_ACCELERATION;

        if (startTimeConstructor) {
            minDuration = StretchedDisplacementCalculator.findMinDuration(distance, MAV, MAA);
            timeSpan = new TimeSpan(startTime, startTime + minDuration);
        }

        // create calculator object
        calculator = new StretchedDisplacementCalculator(distance, timeSpan, MAV, MAA);

        minDuration = calculator.getMinDuration();
    }
}
