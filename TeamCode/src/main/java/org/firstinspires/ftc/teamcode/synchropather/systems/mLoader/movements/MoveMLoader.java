package org.firstinspires.ftc.teamcode.synchropather.systems.mLoader.movements;

import org.firstinspires.ftc.teamcode.synchropather.systems.MovementType;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.TimeSpan;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.Movement;
import org.firstinspires.ftc.teamcode.synchropather.systems.mLoader.MLoaderState;

/**
 * Go to position Movement using the magazine loader.
 */
public class MoveMLoader extends Movement {

    private final double duration = 0.25;
    private final MLoaderState state;

    public MoveMLoader(TimeSpan timeSpan, MLoaderState targetState) {
        super(MovementType.MAGAZINE_LOADER);
        this.state = targetState;
        this.timeSpan = timeSpan;
    }

    public MoveMLoader(TimeSpan timeSpan, double targetPosition) {
        this(timeSpan, new MLoaderState(targetPosition));
    }

    public MoveMLoader(double startTime, MLoaderState targetState) {
        super(MovementType.MAGAZINE_LOADER);
        this.state = targetState;
        timeSpan = new TimeSpan(startTime, startTime + duration);
    }

    public MoveMLoader(double startTime, double targetPosition) {
        this(startTime, new MLoaderState(targetPosition));
    }

    public MoveMLoader(double endTime, double targetPosition, boolean alignToEndTime) {
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
     * @return the indicated MLoaderState.
     */
    @Override
    public MLoaderState getState(double elapsedTime) {
        return state;
    }

    /**
     * @return the indicated velocity MLoaderState.
     */
    @Override
    public MLoaderState getVelocity(double elapsedTime) {
        return new MLoaderState(0);
    }

    /**
     * @return the indicated acceleration MLoaderState.
     */
    @Override
    public MLoaderState getAcceleration(double elapsedTime) {
        return new MLoaderState(0);
    }

    /**
     * @return the MLoaderState of this Movement at the start time.
     */
    @Override
    public MLoaderState getStartState() {
        return state;
    }

    /**
     * @return the MLoaderState of this Movement at the end time.
     */
    @Override
    public MLoaderState getEndState() {
        return state;
    }

    /**
     * @return "MoveMLoader"
     */
    @Override
    public String getDisplayName() {
        return "MoveMLoader";
    }

}
