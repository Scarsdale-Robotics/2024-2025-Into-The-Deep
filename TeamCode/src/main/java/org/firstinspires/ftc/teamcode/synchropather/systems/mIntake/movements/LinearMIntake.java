package org.firstinspires.ftc.teamcode.synchropather.systems.mIntake.movements;

import org.firstinspires.ftc.teamcode.synchropather.systems.MovementType;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.TimeSpan;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.motion_profiles.SymmetricMotionProfile1D;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.Movement;
import org.firstinspires.ftc.teamcode.synchropather.systems.mIntake.MIntakeConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.mIntake.MIntakeState;

/**
 * Linear magazine intake Movement.
 */
public class LinearMIntake extends Movement {
    private double distance, minDuration;
    private MIntakeState start, end;
    private SymmetricMotionProfile1D motionProfile;

    public LinearMIntake(TimeSpan timeSpan, MIntakeState start, MIntakeState end) {
        super(timeSpan, MovementType.MAGAZINE_INTAKE);
        this.start = start;
        this.end = end;
        init(false, 0);
    }

    /**
     * Creates a new LinearMIntake object with a given start and end states at the given startTime.
     * @param startTime
     * @param start
     * @param end
     */
    public LinearMIntake(double startTime, MIntakeState start, MIntakeState end) {
        super(MovementType.MAGAZINE_INTAKE);
        this.start = start;
        this.end = end;
        init(true, startTime);
    }

    /**
     * Creates a new LinearMIntake object with a given start and end MIntakeState at the given startTime.
     * @param endTime
     * @param start
     * @param end
     * @param alignToEndTime if this Movement should end at the given time
     */
    public LinearMIntake(double endTime, MIntakeState start, MIntakeState end, boolean alignToEndTime) {
        super(MovementType.MAGAZINE_INTAKE);
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
     * @return the indicated MIntakeState.
     */
    @Override
    public MIntakeState getState(double elapsedTime) {
        double t = distance!=0 ? motionProfile.getDisplacement(elapsedTime) / distance : 0;

        double q0 = 1 - t;
        double q1 = t;

        // linear interpolation
        return start.times(q0).plus(end.times(q1));
    }

    /**
     * @return the indicated velocity MIntakeState.
     */
    @Override
    public MIntakeState getVelocity(double elapsedTime) {
        double sign = end.minus(start).sign();
        double speed = motionProfile.getVelocity(elapsedTime);

        // scaled velocity vector
        return new MIntakeState(sign * speed);
    }

    /**
     * @return the indicated acceleration MIntakeState.
     */
    @Override
    public MIntakeState getAcceleration(double elapsedTime) {
        double sign = end.minus(start).sign();
        double speed = motionProfile.getAcceleration(elapsedTime);

        // scaled acceleration vector
        return new MIntakeState(sign * speed);
    }

    /**
     * @return the MIntakeState of this Movement at the start time.
     */
    @Override
    public MIntakeState getStartState() {
        return start;
    }

    /**
     * @return the MIntakeState of this Movement at the end time.
     */
    @Override
    public MIntakeState getEndState() {
        return end;
    }

    /**
     * @return "LinearMIntake"
     */
    @Override
    public String getDisplayName() {
        return "LinearMIntake";
    }

    /**
     * Calculates total time.
     */
    private void init(boolean startTimeConstructor, double startTime) {
        distance = end.minus(start).abs();

        double v_max = MIntakeConstants.MAX_VELOCITY;
        double a_max = MIntakeConstants.MAX_ACCELERATION;

        if (startTimeConstructor) {
            motionProfile = new SymmetricMotionProfile1D(distance, startTime, v_max, a_max);
            timeSpan = motionProfile.getTimeSpan();
        } else {
            motionProfile = new SymmetricMotionProfile1D(distance, timeSpan, v_max, a_max);
        }

        minDuration = motionProfile.getMinDuration();
    }
}
