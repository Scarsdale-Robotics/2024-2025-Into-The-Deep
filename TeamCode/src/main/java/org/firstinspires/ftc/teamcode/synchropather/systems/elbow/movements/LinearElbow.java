package org.firstinspires.ftc.teamcode.synchropather.systems.elbow.movements;

import org.firstinspires.ftc.teamcode.synchropather.systems.MovementType;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.TimeSpan;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.calculators.StretchedDisplacementCalculator;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.Movement;
import org.firstinspires.ftc.teamcode.synchropather.systems.elbow.ElbowConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.elbow.ElbowState;

public class LinearElbow extends Movement {
    private double distance, minDuration;
    private ElbowState start, end;
    private StretchedDisplacementCalculator calculator;

    public LinearElbow(TimeSpan timeSpan, ElbowState start, ElbowState end) {
        super(timeSpan, MovementType.ELBOW);
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
    public LinearElbow(double startTime, ElbowState start, ElbowState end) {
        super(MovementType.ELBOW);
        this.start = start;
        this.end = end;
        init(true, startTime);
    }

    @Override
    public double getMinDuration() {
        return minDuration;
    }
    /**
     * @return the indicated ElbowState.
     */
    @Override
    public ElbowState getState(double elapsedTime) {
        double t = distance!=0 ? calculator.getDisplacement(elapsedTime) / distance : 0;

        double q0 = 1 - t;
        double q1 = t;

        // linear interpolation
        return start.times(q0).plus(end.times(q1));
    }
    /**
     * @return the indicated velocity ElbowState.
     */
    @Override
    public ElbowState getVelocity(double elapsedTime) {
        double sign = end.minus(start).sign();
        double speed = calculator.getVelocity(elapsedTime);

        // scaled velocity vector
        return new ElbowState(sign * speed);
    }
    /**
     * @return the indicated acceleration ElbowState.
     */
    @Override
    public ElbowState getAcceleration(double elapsedTime) {
        double sign = end.minus(start).sign();
        double speed = calculator.getAcceleration(elapsedTime);

        // scaled acceleration vector
        return new ElbowState(sign * speed);
    }
    /**
     * @return the ElbowState of this Movement at the start time.
     */
    @Override
    public ElbowState getStartState() {
        return start;
    }
    /**
     * @return the ElbowState of this Movement at the end time.
     */
    @Override
    public ElbowState getEndState() {
        return end;
    }

    /**
     * @return "LinearElbow"
     */
    @Override
    public String getDisplayName() {
        return "LinearElbow";
    }

    /**
     * Calculates total time.
     */
    private void init(boolean startTimeConstructor, double startTime) {
        distance = end.minus(start).abs();

        double MAV = ElbowConstants.MAX_VELOCITY;
        double MAA = ElbowConstants.MAX_ACCELERATION;

        if (startTimeConstructor) {
            minDuration = StretchedDisplacementCalculator.findMinDuration(distance, MAV, MAA);
            timeSpan = new TimeSpan(startTime, startTime + minDuration);
        }

        // create calculator object
        calculator = new StretchedDisplacementCalculator(distance, timeSpan, MAV, MAA);

        minDuration = calculator.getMinDuration();
    }
}
