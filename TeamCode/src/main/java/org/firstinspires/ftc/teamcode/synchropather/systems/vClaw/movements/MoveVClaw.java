package org.firstinspires.ftc.teamcode.synchropather.systems.vClaw.movements;

import static org.firstinspires.ftc.teamcode.synchropather.systems.vClaw.VClawConstants.GRAB_POSITION;
import static org.firstinspires.ftc.teamcode.synchropather.systems.vClaw.VClawConstants.MAX_SPEED;
import static org.firstinspires.ftc.teamcode.synchropather.systems.vClaw.VClawConstants.RELEASE_POSITION;

import org.firstinspires.ftc.teamcode.synchropather.systems.MovementType;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.TimeSpan;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.Movement;
import org.firstinspires.ftc.teamcode.synchropather.systems.vClaw.VClawConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.vClaw.VClawState;

/**
 * Movement using the vertical claw.
 */
public class MoveVClaw extends Movement {

    private final double duration;
    private final VClawState start;
    private final VClawState end;

    public MoveVClaw(double startTime, VClawState start, VClawState end) {
        super(MovementType.VERTICAL_CLAW);
        this.duration = Math.abs(GRAB_POSITION - RELEASE_POSITION) / MAX_SPEED;
        timeSpan = new TimeSpan(startTime, startTime + duration);
        this.start = start;
        this.end = end;
    }

    public MoveVClaw(double startTime, double duration, VClawState start, VClawState end) {
        super(MovementType.VERTICAL_CLAW);
        this.duration = duration;
        timeSpan = new TimeSpan(startTime, startTime + duration);
        this.start = start;
        this.end = end;
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
        return end;
    }

    /**
     * @return the indicated velocity VClawState.
     */
    @Override
    public VClawState getVelocity(double elapsedTime) {
        double sign = Math.signum(end.minus(start).getPosition());
        return new VClawState(sign* MAX_SPEED);
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
        return start;
    }

    /**
     * @return the VClawState of this Movement at the end time.
     */
    @Override
    public VClawState getEndState() {
        return end;
    }

    /**
     * @return "MoveVClaw"
     */
    @Override
    public String getDisplayName() {
        return "MoveVClaw";
    }

}
