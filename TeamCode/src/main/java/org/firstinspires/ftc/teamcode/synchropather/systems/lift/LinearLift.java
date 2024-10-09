package org.firstinspires.ftc.teamcode.synchropather.systems.lift;

import org.firstinspires.ftc.teamcode.synchropather.systems.MovementType;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.TimeSpan;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.calculators.StretchedDisplacementCalculator;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.Movement;

public class LinearLift extends Movement {
    private double distance, minDuration;
    private LiftState start, end;
    private StretchedDisplacementCalculator calculator;

    public LinearLift(TimeSpan timeSpan, LiftState start, LiftState end) {
        super(timeSpan, MovementType.LIFT);
        this.start = start;
        this.end = end;
        init(false, -1);
    }
    /**
     * Creates a new LinearRotation object with a given start and end RotationState at the given startTime.
     * @param startTime
     * @param start
     * @param end
     */
    public LinearLift(double startTime, LiftState start, LiftState end) {
        super(MovementType.LIFT);
        this.start = start;
        this.end = end;
        init(true, startTime);
    }

    @Override
    public double getMinDuration() {
        return minDuration;
    }
    /**
     * @return the indicated LiftState.
     */
    @Override
    public LiftState getState(double elapsedTime) {
        double t = distance!=0 ? calculator.getDisplacement(elapsedTime) / distance : 0;

        double q0 = 1 - t;
        double q1 = t;

        // linear interpolation
        return start.times(q0).plus(end.times(q1));
    }
    /**
     * @return the indicated velocity LiftState.
     */
    @Override
    public LiftState getVelocity(double elapsedTime) {
        double sign = end.minus(start).sign();
        double speed = calculator.getVelocity(elapsedTime);

        // scaled velocity vector
        return new LiftState(sign * speed);
    }
    /**
     * @return the indicated acceleration LiftState.
     */
    @Override
    public LiftState getAcceleration(double elapsedTime) {
        double sign = end.minus(start).sign();
        double speed = calculator.getAcceleration(elapsedTime);

        // scaled acceleration vector
        return new LiftState(sign * speed);
    }
    /**
     * @return the LiftState of this Movement at the start time.
     */
    @Override
    public LiftState getStartState() {
        return start;
    }
    /**
     * @return the LiftState of this Movement at the end time.
     */
    @Override
    public LiftState getEndState() {
        return end;
    }

    /**
     * @return "LinearLift"
     */
    @Override
    public String getDisplayName() {
        return "LinearLift";
    }

    /**
     * Calculates total time.
     */
    private void init(boolean startTimeConstructor, double startTime) {
        distance = end.minus(start).abs();

        double MAV = LiftConstants.MAX_VELOCITY;
        double MAA = LiftConstants.MAX_ACCELERATION;

        if (startTimeConstructor) {
            minDuration = StretchedDisplacementCalculator.findMinDuration(distance, MAV, MAA);
            timeSpan = new TimeSpan(startTime, startTime + minDuration);
        }

        // create calculator object
        calculator = new StretchedDisplacementCalculator(distance, timeSpan, MAV, MAA);

        minDuration = calculator.getMinDuration();
    }
}
