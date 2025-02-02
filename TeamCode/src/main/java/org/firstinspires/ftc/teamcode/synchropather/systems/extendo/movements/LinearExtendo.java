package org.firstinspires.ftc.teamcode.synchropather.systems.extendo.movements;

import org.firstinspires.ftc.teamcode.synchropather.systems.MovementType;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.TimeSpan;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.calculators.StretchedDisplacementCalculator;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.Movement;
import org.firstinspires.ftc.teamcode.synchropather.systems.extendo.ExtendoConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.extendo.ExtendoState;

public class LinearExtendo extends Movement {
    private double distance, minDuration;
    private ExtendoState start, end;
    private StretchedDisplacementCalculator calculator;

    public LinearExtendo(TimeSpan timeSpan, ExtendoState start, ExtendoState end) {
        super(timeSpan, MovementType.EXTENDO);
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
    public LinearExtendo(double startTime, ExtendoState start, ExtendoState end) {
        super(MovementType.EXTENDO);
        this.start = start;
        this.end = end;
        init(true, startTime);
    }

    @Override
    public double getMinDuration() {
        return minDuration;
    }
    /**
     * @return the indicated ExtendoState.
     */
    @Override
    public ExtendoState getState(double elapsedTime) {
        double t = distance!=0 ? calculator.getDisplacement(elapsedTime) / distance : 0;

        double q0 = 1 - t;
        double q1 = t;

        // linear interpolation
        return start.times(q0).plus(end.times(q1));
    }
    /**
     * @return the indicated velocity ExtendoState.
     */
    @Override
    public ExtendoState getVelocity(double elapsedTime) {
        double sign = end.minus(start).sign();
        double speed = calculator.getVelocity(elapsedTime);

        // scaled velocity vector
        return new ExtendoState(sign * speed);
    }
    /**
     * @return the indicated acceleration ExtendoState.
     */
    @Override
    public ExtendoState getAcceleration(double elapsedTime) {
        double sign = end.minus(start).sign();
        double speed = calculator.getAcceleration(elapsedTime);

        // scaled acceleration vector
        return new ExtendoState(sign * speed);
    }
    /**
     * @return the ExtendoState of this Movement at the start time.
     */
    @Override
    public ExtendoState getStartState() {
        return start;
    }
    /**
     * @return the ExtendoState of this Movement at the end time.
     */
    @Override
    public ExtendoState getEndState() {
        return end;
    }

    /**
     * @return "LinearExtendo"
     */
    @Override
    public String getDisplayName() {
        return "LinearExtendo";
    }

    /**
     * Calculates total time.
     */
    private void init(boolean startTimeConstructor, double startTime) {
        distance = end.minus(start).abs();

        double MAV = ExtendoConstants.MAX_VELOCITY;
        double MAA = ExtendoConstants.MAX_ACCELERATION;

        if (startTimeConstructor) {
            minDuration = StretchedDisplacementCalculator.findMinDuration(distance, MAV, MAA);
            timeSpan = new TimeSpan(startTime, startTime + minDuration);
        }

        // create calculator object
        calculator = new StretchedDisplacementCalculator(distance, timeSpan, MAV, MAA);

        minDuration = calculator.getMinDuration();
    }
}
