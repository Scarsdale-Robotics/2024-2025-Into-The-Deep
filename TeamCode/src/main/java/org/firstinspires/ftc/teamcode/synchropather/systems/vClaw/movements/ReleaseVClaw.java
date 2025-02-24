package org.firstinspires.ftc.teamcode.synchropather.systems.vClaw.movements;

import org.firstinspires.ftc.teamcode.synchropather.systems.MovementType;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.TimeSpan;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.Movement;
import org.firstinspires.ftc.teamcode.synchropather.systems.vClaw.VClawConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.vClaw.VClawState;

/**
 * Release Movement using the vertical claw.
 */
public class ReleaseVClaw extends Movement {

    private final double duration = VClawConstants.MOVEMENT_TIME;
    private final VClawState state = new VClawState(VClawConstants.RELEASE_POSITION);

    public ReleaseVClaw(double startTime) {
        super(MovementType.VERTICAL_CLAW);
        timeSpan = new TimeSpan(startTime, startTime + duration);
    }

    public ReleaseVClaw(double endTime, boolean alignToEndTime) {
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
     * @return the indicated VClawState.
     */
    @Override
    public VClawState getState(double elapsedTime) {
        double clampedElapsedTime = timeSpan.clamp(elapsedTime)-getStartTime();
        if (clampedElapsedTime<=0) return getStartState();
        return state;
    }

    /**
     * @return the indicated velocity VClawState.
     */
    @Override
    public VClawState getVelocity(double elapsedTime) {
        double sign = Math.signum(VClawConstants.RELEASE_POSITION - VClawConstants.GRAB_POSITION);
        return new VClawState(sign* VClawConstants.MAX_SPEED);
    }
    /**
     * @return the indicated acceleration VClawState.
     */
    @Override
    public VClawState getAcceleration(double elapsedTime) {
        return new VClawState(0);
    }
    /**
     * @return the VClawState of this Movement at the start time.
     */
    @Override
    public VClawState getStartState() {
        return new VClawState(VClawConstants.GRAB_POSITION);
    }

    /**
     * @return the VClawState of this Movement at the end time.
     */
    @Override
    public VClawState getEndState() {
        return state;
    }

    /**
     * @return "ReleaseVClaw"
     */
    @Override
    public String getDisplayName() {
        return "ReleaseVClaw";
    }

}
