package org.firstinspires.ftc.teamcode.synchropather.systems.extendo.movements;

import org.firstinspires.ftc.teamcode.synchropather.systems.MovementType;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.TimeSpan;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.motion_profiles.DynamicMotionProfile1D;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.motion_profiles.MotionProfile1D;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.Movement;
import org.firstinspires.ftc.teamcode.synchropather.systems.extendo.ExtendoConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.extendo.ExtendoState;

public class DynamicLinearExtendo extends Movement {
    private double distance, minDuration;
    private ExtendoState start, end, v0;
    private MotionProfile1D motionProfile;

    public DynamicLinearExtendo(TimeSpan timeSpan, ExtendoState start, ExtendoState end, ExtendoState v0) {
        super(timeSpan, MovementType.EXTENDO);
        this.start = start;
        this.end = end;
        this.v0 = v0;
        init(false, 0);
    }
    /**
     * Creates a new LinearRotation object with a given start and end RotationState at the given startTime.
     * @param startTime
     * @param start
     * @param end
     */
    public DynamicLinearExtendo(double startTime, ExtendoState start, ExtendoState end, ExtendoState v0) {
        super(MovementType.EXTENDO);
        this.start = start;
        this.end = end;
        this.v0 = v0;
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
        double t = distance!=0 ? motionProfile.getDisplacement(elapsedTime) / distance : 0;

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
        double speed = motionProfile.getVelocity(elapsedTime);

        // scaled velocity vector
        return new ExtendoState(sign * speed);
    }
    /**
     * @return the indicated acceleration ExtendoState.
     */
    @Override
    public ExtendoState getAcceleration(double elapsedTime) {
        double sign = end.minus(start).sign();
        double speed = motionProfile.getAcceleration(elapsedTime);

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

        double v0 = this.v0.getLength();
        double v_max = ExtendoConstants.MAX_VELOCITY;
        double a_max = ExtendoConstants.MAX_ACCELERATION;

        if (startTimeConstructor) {
            motionProfile = new DynamicMotionProfile1D(distance, startTime, v0, v_max, a_max, a_max);
            timeSpan = motionProfile.getTimeSpan();
        } else {
            motionProfile = new DynamicMotionProfile1D(distance, timeSpan, v0, v_max, a_max, a_max);
        }

        minDuration = motionProfile.getMinDuration();

    }
}
