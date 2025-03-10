package org.firstinspires.ftc.teamcode.synchropather.systems.mIntake.movements;

import org.firstinspires.ftc.teamcode.synchropather.systems.MovementType;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.TimeSpan;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.Movement;
import org.firstinspires.ftc.teamcode.synchropather.systems.mIntake.MIntakeState;

/**
 * Go to position Movement using the magazine intake.
 */
public class MoveMIntake extends Movement {

    private final double duration = 0.25;
    private final MIntakeState state;

    public MoveMIntake(TimeSpan timeSpan, MIntakeState targetState) {
        super(MovementType.MAGAZINE_INTAKE);
        this.state = targetState;
        this.timeSpan = timeSpan;
    }

    public MoveMIntake(TimeSpan timeSpan, double targetPosition) {
        this(timeSpan, new MIntakeState(targetPosition));
    }

    public MoveMIntake(double startTime, MIntakeState targetState) {
        super(MovementType.MAGAZINE_INTAKE);
        this.state = targetState;
        timeSpan = new TimeSpan(startTime, startTime + duration);
    }

    public MoveMIntake(double startTime, double targetPosition) {
        this(startTime, new MIntakeState(targetPosition));
    }

    public MoveMIntake(double endTime, double targetPosition, boolean alignToEndTime) {
        this(endTime, targetPosition);
        if (alignToEndTime) {
            double startTime = Math.max(0, endTime-duration);
            this.timeSpan = new TimeSpan(startTime, startTime+duration);
        }
    }

    @Override
    public double getMinDuration() {
        return duration;
    }

    /**
     * @return the indicated MIntakeState.
     */
    @Override
    public MIntakeState getState(double elapsedTime) {
        return state;
    }

    /**
     * @return the indicated velocity MIntakeState.
     */
    @Override
    public MIntakeState getVelocity(double elapsedTime) {
        return new MIntakeState(0);
    }

    /**
     * @return the indicated acceleration MIntakeState.
     */
    @Override
    public MIntakeState getAcceleration(double elapsedTime) {
        return new MIntakeState(0);
    }

    /**
     * @return the MIntakeState of this Movement at the start time.
     */
    @Override
    public MIntakeState getStartState() {
        return state;
    }

    /**
     * @return the MIntakeState of this Movement at the end time.
     */
    @Override
    public MIntakeState getEndState() {
        return state;
    }

    /**
     * @return "MoveMIntake"
     */
    @Override
    public String getDisplayName() {
        return "MoveMIntake";
    }

}
