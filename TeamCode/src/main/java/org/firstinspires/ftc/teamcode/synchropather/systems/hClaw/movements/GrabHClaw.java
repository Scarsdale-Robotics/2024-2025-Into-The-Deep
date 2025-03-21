package org.firstinspires.ftc.teamcode.synchropather.systems.hClaw.movements;

import org.firstinspires.ftc.teamcode.synchropather.systems.MovementType;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.TimeSpan;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.Movement;
import org.firstinspires.ftc.teamcode.synchropather.systems.hClaw.HClawConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.hClaw.HClawState;

/**
 * Grab Movement using the horizontal claw.
 */
public class GrabHClaw extends Movement {

    private final double duration;
    private final HClawState state = new HClawState(HClawConstants.GRAB_POSITION);

    public GrabHClaw(double startTime) {
        super(MovementType.HORIZONTAL_CLAW);
        this.duration = HClawConstants.MOVEMENT_TIME;
        timeSpan = new TimeSpan(startTime, startTime + duration);
    }

    public GrabHClaw(double startTime, double duration) {
        super(MovementType.HORIZONTAL_CLAW);
        this.duration = duration;
        timeSpan = new TimeSpan(startTime, startTime + duration);
    }

    public GrabHClaw(double endTime, boolean alignToEndTime) {
        this(endTime);
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
        double sign = Math.signum(HClawConstants.GRAB_POSITION-HClawConstants.RELEASE_POSITION);
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
        return new HClawState(HClawConstants.RELEASE_POSITION);
    }

    /**
     * @return the HClawState of this Movement at the end time.
     */
    @Override
    public HClawState getEndState() {
        return state;
    }

    /**
     * @return "GrabHClaw"
     */
    @Override
    public String getDisplayName() {
        return "GrabHClaw";
    }

}
