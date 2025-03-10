package org.firstinspires.ftc.teamcode.synchropather.systems.mFeeder.movements;

import org.firstinspires.ftc.teamcode.synchropather.systems.MovementType;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.TimeSpan;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.motion_profiles.SymmetricMotionProfile1D;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.Movement;
import org.firstinspires.ftc.teamcode.synchropather.systems.mFeeder.MFeederConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.mFeeder.MFeederState;

public class LinearMFeeder extends Movement {
    private double distance, minDuration;
    private MFeederState start, end;
    private SymmetricMotionProfile1D motionProfile;

    public LinearMFeeder(TimeSpan timeSpan, MFeederState start, MFeederState end) {
        super(timeSpan, MovementType.MAGAZINE_FEEDER);
        this.start = start;
        this.end = end;
        init(false, 0);
    }

    /**
     * Creates a new LinearMFeeder object with a given start and end MFeederState at the given startTime.
     * @param startTime
     * @param start
     * @param end
     */
    public LinearMFeeder(double startTime, MFeederState start, MFeederState end) {
        super(MovementType.MAGAZINE_FEEDER);
        this.start = start;
        this.end = end;
        init(true, startTime);
    }

    /**
     * Creates a new LinearMFeeder object with a given start and end MFeederState at the given startTime.
     * @param endTime
     * @param start
     * @param end
     * @param alignToEndTime if this Movement should end at the given time
     */
    public LinearMFeeder(double endTime, MFeederState start, MFeederState end, boolean alignToEndTime) {
        super(MovementType.MAGAZINE_FEEDER);
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
     * @return the indicated MFeederState.
     */
    @Override
    public MFeederState getState(double elapsedTime) {
        double t = distance!=0 ? motionProfile.getDisplacement(elapsedTime) / distance : 0;

        double q0 = 1 - t;
        double q1 = t;

        // linear interpolation
        return start.times(q0).plus(end.times(q1));
    }
    /**
     * @return the indicated velocity MFeederState.
     */
    @Override
    public MFeederState getVelocity(double elapsedTime) {
        double sign = end.minus(start).sign();
        double speed = motionProfile.getVelocity(elapsedTime);

        // scaled velocity vector
        return new MFeederState(sign * speed);
    }
    /**
     * @return the indicated acceleration MFeederState.
     */
    @Override
    public MFeederState getAcceleration(double elapsedTime) {
        double sign = end.minus(start).sign();
        double speed = motionProfile.getAcceleration(elapsedTime);

        // scaled acceleration vector
        return new MFeederState(sign * speed);
    }
    /**
     * @return the MFeederState of this Movement at the start time.
     */
    @Override
    public MFeederState getStartState() {
        return start;
    }
    /**
     * @return the MFeederState of this Movement at the end time.
     */
    @Override
    public MFeederState getEndState() {
        return end;
    }

    /**
     * @return "LinearMFeeder"
     */
    @Override
    public String getDisplayName() {
        return "LinearMFeeder";
    }

    /**
     * Calculates total time.
     */
    private void init(boolean startTimeConstructor, double startTime) {
        distance = end.minus(start).abs();

        double v_max = MFeederConstants.MAX_VELOCITY;
        double a_max = MFeederConstants.MAX_ACCELERATION;

        if (startTimeConstructor) {
            motionProfile = new SymmetricMotionProfile1D(distance, startTime, v_max, a_max);
            timeSpan = motionProfile.getTimeSpan();
        } else {
            motionProfile = new SymmetricMotionProfile1D(distance, timeSpan, v_max, a_max);
        }

        minDuration = motionProfile.getMinDuration();

    }
}
