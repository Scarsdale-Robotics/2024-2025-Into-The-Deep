package org.firstinspires.ftc.teamcode.synchropather.systems.hClaw.movements;

import org.firstinspires.ftc.teamcode.synchropather.systems.MovementType;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.TimeSpan;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.Movement;
import org.firstinspires.ftc.teamcode.synchropather.systems.hClaw.HClawConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.hClaw.HClawState;

/**
 * Release Movement using the horizontal claw.
 */
public class ReleaseHClaw extends Movement {

    private final double duration = HClawConstants.MOVEMENT_TIME;
    private final HClawState state = new HClawState(HClawConstants.RELEASE_POSITION);

    public ReleaseHClaw(double startTime) {
        super(MovementType.HORIZONTAL_CLAW);
        timeSpan = new TimeSpan(startTime, startTime + duration);
    }

    public ReleaseHClaw(double endTime, boolean alignToEndTime) {
        this(endTime);
        if (alignToEndTime) {
            double startTime = Math.max(0, endTime - duration);
            timeSpan = new TimeSpan(startTime, startTime + duration);
        }
    }

    @Override
    public double getMinDuration() {
        return duration;
    }

    /**
     * @return the indicated HClawState.
     */
    @Override
    public HClawState getState(double elapsedTime) {
        double clampedElapsedTime = timeSpan.clamp(elapsedTime)-getStartTime();
        if (clampedElapsedTime<=0) return getStartState();
        return state;
    }

    /**
     * @return the indicated velocity HClawState.
     */
    @Override
    public HClawState getVelocity(double elapsedTime) {
        double sign = Math.signum(HClawConstants.RELEASE_POSITION - HClawConstants.GRAB_POSITION);
        return new HClawState(sign*HClawConstants.MAX_SPEED);
    }
    /**
     * @return the indicated acceleration HClawState.
     */
    @Override
    public HClawState getAcceleration(double elapsedTime) {
        return new HClawState(0);
    }
    /**
     * @return the HClawState of this Movement at the start time.
     */
    @Override
    public HClawState getStartState() {
        return new HClawState(HClawConstants.GRAB_POSITION);
    }

    /**
     * @return the HClawState of this Movement at the end time.
     */
    @Override
    public HClawState getEndState() {
        return state;
    }

    /**
     * @return "ReleaseHClaw"
     */
    @Override
    public String getDisplayName() {
        return "ReleaseHClaw";
    }

}
