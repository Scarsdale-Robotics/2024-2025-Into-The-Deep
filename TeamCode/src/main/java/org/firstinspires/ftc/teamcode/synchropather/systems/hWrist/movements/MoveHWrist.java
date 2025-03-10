package org.firstinspires.ftc.teamcode.synchropather.systems.hWrist.movements;

import org.firstinspires.ftc.teamcode.synchropather.systems.MovementType;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.TimeSpan;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.Movement;
import org.firstinspires.ftc.teamcode.synchropather.systems.hWrist.HWristState;

/**
 * Go to position Movement using the horizontal wrist.
 */
public class MoveHWrist extends Movement {

    private final double duration = 0.1;
    private final HWristState state;

    public MoveHWrist(double startTime, HWristState targetState) {
        super(MovementType.HORIZONTAL_WRIST);
        this.state = targetState;
        timeSpan = new TimeSpan(startTime, startTime + duration);
    }

    public MoveHWrist(double startTime, double targetAngle) {
        this(startTime, new HWristState(targetAngle));
    }

    public MoveHWrist(double endTime, double targetAngle, boolean alignToEndTime) {
        this(endTime, targetAngle);
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
     * @return the indicated HWristState.
     */
    @Override
    public HWristState getState(double elapsedTime) {
        return state;
    }

    /**
     * @return the indicated velocity HWristState.
     */
    @Override
    public HWristState getVelocity(double elapsedTime) {
        return new HWristState(0);
    }
    /**
     * @return the indicated acceleration HWristState.
     */
    @Override
    public HWristState getAcceleration(double elapsedTime) {
        return new HWristState(0);
    }
    /**
     * @return the HWristState of this Movement at the start time.
     */
    @Override
    public HWristState getStartState() {
        return state;
    }

    /**
     * @return the HWristState of this Movement at the end time.
     */
    @Override
    public HWristState getEndState() {
        return state;
    }

    /**
     * @return "MoveHWrist"
     */
    @Override
    public String getDisplayName() {
        return "MoveHWrist";
    }

}
