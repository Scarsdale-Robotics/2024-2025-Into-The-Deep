package org.firstinspires.ftc.teamcode.synchropather.systems.lift.movements;

import org.firstinspires.ftc.teamcode.synchropather.systems.MovementType;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.TimeSpan;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.motion_profiles.MotionProfile1D;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.motion_profiles.SymmetricMotionProfile1D;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.motion_profiles.TrapezoidalMotionProfile1D;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.Movement;
import org.firstinspires.ftc.teamcode.synchropather.systems.lift.LiftConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.lift.LiftState;

public class LinearLift extends Movement {
    private double distance, minDuration;
    private LiftState start, end;
    private MotionProfile1D motionProfile;

    /**
     * Creates a new LinearLift object with a given start and end LiftState throughout the given TimeSpan.
     * @param timeSpan
     * @param start
     * @param end
     */
    public LinearLift(TimeSpan timeSpan, LiftState start, LiftState end) {
        super(timeSpan, MovementType.LIFT);
        this.start = start;
        this.end = end;
        init(false, 0, false);
    }

    /**
     * Creates a new LinearLift object with a given start and end LiftState at the given startTime.
     * @param startTime
     * @param start
     * @param end
     */
    public LinearLift(double startTime, LiftState start, LiftState end) {
        super(MovementType.LIFT);
        this.start = start;
        this.end = end;
        init(true, startTime, false);
    }

    /**
     * Creates a new LinearLift object with a given start and end LiftState throughout the given TimeSpan.
     * @param timeSpan
     * @param start
     * @param end
     * @param useDifferentDeceleration whether to use a trapezoidal motion profile
     */
    public LinearLift(TimeSpan timeSpan, LiftState start, LiftState end, boolean useDifferentDeceleration) {
        super(timeSpan, MovementType.LIFT);
        this.start = start;
        this.end = end;
        init(false, 0, useDifferentDeceleration);
    }

    /**
     * Creates a new LinearLift object with a given start and end LiftState at the given startTime.
     * @param startTime
     * @param start
     * @param end
     * @param useDifferentDeceleration whether to use a trapezoidal motion profile
     */
    public LinearLift(double startTime, LiftState start, LiftState end, boolean useDifferentDeceleration) {
        super(MovementType.LIFT);
        this.start = start;
        this.end = end;
        init(true, startTime, useDifferentDeceleration);
    }

    @Override
    public double getMinDuration() {
        return minDuration;
    }
    /**
     * @return the indicated LiftState.
     */
    @Override
    public LiftState getState(double elapsedTime) {
        double t = distance!=0 ? motionProfile.getDisplacement(elapsedTime) / distance : 0;

        double q0 = 1 - t;
        double q1 = t;

        // linear interpolation
        return start.times(q0).plus(end.times(q1));
    }
    /**
     * @return the indicated velocity LiftState.
     */
    @Override
    public LiftState getVelocity(double elapsedTime) {
        double sign = end.minus(start).sign();
        double speed = motionProfile.getVelocity(elapsedTime);

        // scaled velocity vector
        return new LiftState(sign * speed);
    }
    /**
     * @return the indicated acceleration LiftState.
     */
    @Override
    public LiftState getAcceleration(double elapsedTime) {
        double sign = end.minus(start).sign();
        double speed = motionProfile.getAcceleration(elapsedTime);

        // scaled acceleration vector
        return new LiftState(sign * speed);
    }
    /**
     * @return the LiftState of this Movement at the start time.
     */
    @Override
    public LiftState getStartState() {
        return start;
    }
    /**
     * @return the LiftState of this Movement at the end time.
     */
    @Override
    public LiftState getEndState() {
        return end;
    }

    /**
     * @return "LinearLift"
     */
    @Override
    public String getDisplayName() {
        return "LinearLift";
    }

    /**
     * Calculates total time.
     */
    private void init(boolean startTimeConstructor, double startTime, boolean useDifferentDeceleration) {
        distance = end.minus(start).abs();

        double v_max = LiftConstants.MAX_PATHING_VELOCITY;
        double a_max_1 = LiftConstants.MAX_ACCELERATION;
        double a_max_2 = LiftConstants.MAX_DECELERATION;

        if (startTimeConstructor) {
            if (useDifferentDeceleration) {
                motionProfile = new TrapezoidalMotionProfile1D(distance, startTime, v_max, a_max_1, a_max_2);
            } else {
                motionProfile = new SymmetricMotionProfile1D(distance, startTime, v_max, a_max_1);
            }
            timeSpan = motionProfile.getTimeSpan();
        } else {
            if (useDifferentDeceleration) {
                motionProfile = new TrapezoidalMotionProfile1D(distance, timeSpan, v_max, a_max_1, a_max_2);
            } else {
                motionProfile = new SymmetricMotionProfile1D(distance, timeSpan, v_max, a_max_1);
            }
        }

        minDuration = motionProfile.getMinDuration();
    }
}
