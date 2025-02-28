package org.firstinspires.ftc.teamcode.synchropather.systems.klipper.movements;

import org.firstinspires.ftc.teamcode.synchropather.systems.MovementType;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.TimeSpan;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.Movement;
import org.firstinspires.ftc.teamcode.synchropather.systems.klipper.KlipperConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.klipper.KlipperState;

/**
 * Go to position Movement using the klipper.
 */
public class MoveKlipper extends Movement {

    private final double duration = KlipperConstants.MOVEMENT_DURATION;
    private final KlipperState state;

    public MoveKlipper(TimeSpan timeSpan, KlipperState targetState) {
        super(MovementType.KLIPPER);
        this.state = targetState;
        this.timeSpan = timeSpan;
    }

    public MoveKlipper(TimeSpan timeSpan, double targetPosition) {
        this(timeSpan, new KlipperState(targetPosition));
    }

    public MoveKlipper(double startTime, KlipperState targetState) {
        super(MovementType.KLIPPER);
        this.state = targetState;
        timeSpan = new TimeSpan(startTime, startTime + duration);
    }

    public MoveKlipper(double startTime, double targetPosition) {
        this(startTime, new KlipperState(targetPosition));
    }

    public MoveKlipper(double endTime, double targetPosition, boolean alignToEndTime) {
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
     * @return the indicated KlipperState.
     */
    @Override
    public KlipperState getState(double elapsedTime) {
        return state;
    }

    /**
     * @return the indicated velocity KlipperState.
     */
    @Override
    public KlipperState getVelocity(double elapsedTime) {
        return new KlipperState(0);
    }

    /**
     * @return the indicated acceleration KlipperState.
     */
    @Override
    public KlipperState getAcceleration(double elapsedTime) {
        return new KlipperState(0);
    }

    /**
     * @return the KlipperState of this Movement at the start time.
     */
    @Override
    public KlipperState getStartState() {
        return state;
    }

    /**
     * @return the KlipperState of this Movement at the end time.
     */
    @Override
    public KlipperState getEndState() {
        return state;
    }

    /**
     * @return "MoveKlipper"
     */
    @Override
    public String getDisplayName() {
        return "MoveKlipper";
    }

}
