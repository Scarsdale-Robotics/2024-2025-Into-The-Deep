package org.firstinspires.ftc.teamcode.synchropather.systems.extendo.movements;

import org.firstinspires.ftc.teamcode.synchropather.systems.MovementType;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.TimeSpan;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.motion_profiles.SymmetricMotionProfile1D;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.Movement;
import org.firstinspires.ftc.teamcode.synchropather.systems.extendo.ExtendoConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.extendo.ExtendoState;

public class LinearExtendo extends Movement {
    private double distance, minDuration;
    private ExtendoState start, end;
    private SymmetricMotionProfile1D motionProfile;

    public LinearExtendo(TimeSpan timeSpan, ExtendoState start, ExtendoState end) {
        super(timeSpan, MovementType.EXTENDO);
        this.start = start;
        this.end = end;
        init(false, 0);
    }

    /**
     * Creates a new LinearRotation object with a given start and end RotationState at the given startTime.
     * @param startTime
     * @param start
     * @param end
     */
    public LinearExtendo(double startTime, ExtendoState start, ExtendoState end) {
        super(MovementType.EXTENDO);
        this.start = start;
        this.end = end;
        init(true, startTime);
    }

    /**
     * Creates a new LinearExtendo object with a given start and end ExtendoState at the given startTime.
     * @param endTime
     * @param start
     * @param end
     * @param alignToEndTime if this Movement should end at the given time
     */
    public LinearExtendo(double endTime, ExtendoState start, ExtendoState end, boolean alignToEndTime) {
        super(MovementType.EXTENDO);
        this.start = start;
        this.end = end;
        init(true, endTime);
        if (alignToEndTime) {
            double startTime = Math.max(0, endTime-minDuration);
            this.timeSpan = new TimeSpan(startTime, startTime+minDuration);
        }
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

        double v_max = ExtendoConstants.MAX_VELOCITY;
        double a_max = ExtendoConstants.MAX_ACCELERATION;

        if (startTimeConstructor) {
            motionProfile = new SymmetricMotionProfile1D(distance, startTime, v_max, a_max);
            timeSpan = motionProfile.getTimeSpan();
        } else {
            motionProfile = new SymmetricMotionProfile1D(distance, timeSpan, v_max, a_max);
        }

        minDuration = motionProfile.getMinDuration();

    }
}
