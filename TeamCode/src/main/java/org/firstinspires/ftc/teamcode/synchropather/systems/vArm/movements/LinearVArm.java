package org.firstinspires.ftc.teamcode.synchropather.systems.vArm.movements;

import org.firstinspires.ftc.teamcode.synchropather.systems.MovementType;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.TimeSpan;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.motion_profiles.SymmetricMotionProfile1D;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.Movement;
import org.firstinspires.ftc.teamcode.synchropather.systems.vArm.VArmConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.vArm.VArmState;

/**
 * Linear vertical arm Movement.
 */
public class LinearVArm extends Movement {
    private double distance, minDuration;
    private VArmState start, end;
    private SymmetricMotionProfile1D motionProfile;

    public LinearVArm(TimeSpan timeSpan, VArmState start, VArmState end) {
        super(timeSpan, MovementType.VERTICAL_ARM);
        this.start = start;
        this.end = end;
        init(false, 0);
    }

    /**
     * Creates a new LinearVArm object with a given start and end states at the given startTime.
     * @param startTime
     * @param start
     * @param end
     */
    public LinearVArm(double startTime, VArmState start, VArmState end) {
        super(MovementType.VERTICAL_ARM);
        this.start = start;
        this.end = end;
        init(true, startTime);
    }

    /**
     * Creates a new LinearVArm object with a given start and end HArmState at the given startTime.
     * @param endTime
     * @param start
     * @param end
     * @param alignToEndTime if this Movement should end at the given time
     */
    public LinearVArm(double endTime, VArmState start, VArmState end, boolean alignToEndTime) {
        super(MovementType.VERTICAL_ARM);
        this.start = start;
        this.end = end;
        init(true, endTime);
        if (alignToEndTime) {
            double startTime = Math.max(0, endTime-minDuration);
            init(true, startTime);
        }
    }

    @Override
    public double getMinDuration() {
        return minDuration;
    }

    /**
     * @return the indicated HArmState.
     */
    @Override
    public VArmState getState(double elapsedTime) {
        double t = distance!=0 ? motionProfile.getDisplacement(elapsedTime) / distance : 0;

        double q0 = 1 - t;
        double q1 = t;

        // linear interpolation
        return start.times(q0).plus(end.times(q1));
    }

    /**
     * @return the indicated velocity HArmState.
     */
    @Override
    public VArmState getVelocity(double elapsedTime) {
        double sign = end.minus(start).sign();
        double speed = motionProfile.getVelocity(elapsedTime);

        // scaled velocity vector
        return new VArmState(sign * speed);
    }

    /**
     * @return the indicated acceleration HArmState.
     */
    @Override
    public VArmState getAcceleration(double elapsedTime) {
        double sign = end.minus(start).sign();
        double speed = motionProfile.getAcceleration(elapsedTime);

        // scaled acceleration vector
        return new VArmState(sign * speed);
    }

    /**
     * @return the HArmState of this Movement at the start time.
     */
    @Override
    public VArmState getStartState() {
        return start;
    }

    /**
     * @return the HArmState of this Movement at the end time.
     */
    @Override
    public VArmState getEndState() {
        return end;
    }

    /**
     * @return "LinearVArm"
     */
    @Override
    public String getDisplayName() {
        return "LinearVArm";
    }

    /**
     * Calculates total time.
     */
    private void init(boolean startTimeConstructor, double startTime) {
        distance = end.minus(start).abs();

        double v_max = VArmConstants.MAX_VELOCITY;
        double a_max = VArmConstants.MAX_ACCELERATION;

        if (startTimeConstructor) {
            motionProfile = new SymmetricMotionProfile1D(distance, startTime, v_max, a_max);
            timeSpan = motionProfile.getTimeSpan();
        } else {
            motionProfile = new SymmetricMotionProfile1D(distance, timeSpan, v_max, a_max);
        }

        minDuration = motionProfile.getMinDuration();
    }
}
