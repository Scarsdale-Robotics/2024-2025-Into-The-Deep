package org.firstinspires.ftc.teamcode.synchropather.systems.magazine.movements;

import org.firstinspires.ftc.teamcode.synchropather.systems.MovementType;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.TimeSpan;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.motion_profiles.SymmetricMotionProfile1D;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.Movement;
import org.firstinspires.ftc.teamcode.synchropather.systems.magazine.MagazineConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.magazine.MagazineState;

public class LinearMagazine extends Movement {
    private double distance, minDuration;
    private MagazineState start, end;
    private SymmetricMotionProfile1D motionProfile;

    public LinearMagazine(TimeSpan timeSpan, MagazineState start, MagazineState end) {
        super(timeSpan, MovementType.CLIPBOT_MAGAZINE);
        this.start = start;
        this.end = end;
        init(false, 0);
    }

    /**
     * Creates a new LinearMagazine object with a given start and end MagazineState at the given startTime.
     * @param startTime
     * @param start
     * @param end
     */
    public LinearMagazine(double startTime, MagazineState start, MagazineState end) {
        super(MovementType.CLIPBOT_MAGAZINE);
        this.start = start;
        this.end = end;
        init(true, startTime);
    }

    /**
     * Creates a new LinearMagazine object with a given start and end MagazineState at the given startTime.
     * @param endTime
     * @param start
     * @param end
     * @param alignToEndTime if this Movement should end at the given time
     */
    public LinearMagazine(double endTime, MagazineState start, MagazineState end, boolean alignToEndTime) {
        super(MovementType.CLIPBOT_MAGAZINE);
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
     * @return the indicated MagazineState.
     */
    @Override
    public MagazineState getState(double elapsedTime) {
        double t = distance!=0 ? motionProfile.getDisplacement(elapsedTime) / distance : 0;

        double q0 = 1 - t;
        double q1 = t;

        // linear interpolation
        return start.times(q0).plus(end.times(q1));
    }
    /**
     * @return the indicated velocity MagazineState.
     */
    @Override
    public MagazineState getVelocity(double elapsedTime) {
        double sign = end.minus(start).sign();
        double speed = motionProfile.getVelocity(elapsedTime);

        // scaled velocity vector
        return new MagazineState(sign * speed);
    }
    /**
     * @return the indicated acceleration MagazineState.
     */
    @Override
    public MagazineState getAcceleration(double elapsedTime) {
        double sign = end.minus(start).sign();
        double speed = motionProfile.getAcceleration(elapsedTime);

        // scaled acceleration vector
        return new MagazineState(sign * speed);
    }
    /**
     * @return the MagazineState of this Movement at the start time.
     */
    @Override
    public MagazineState getStartState() {
        return start;
    }
    /**
     * @return the MagazineState of this Movement at the end time.
     */
    @Override
    public MagazineState getEndState() {
        return end;
    }

    /**
     * @return "LinearMagazine"
     */
    @Override
    public String getDisplayName() {
        return "LinearMagazine";
    }

    /**
     * Calculates total time.
     */
    private void init(boolean startTimeConstructor, double startTime) {
        distance = end.minus(start).abs();

        double v_max = MagazineConstants.MAX_VELOCITY;
        double a_max = MagazineConstants.MAX_ACCELERATION;

        if (startTimeConstructor) {
            motionProfile = new SymmetricMotionProfile1D(distance, startTime, v_max, a_max);
            timeSpan = motionProfile.getTimeSpan();
        } else {
            motionProfile = new SymmetricMotionProfile1D(distance, timeSpan, v_max, a_max);
        }

        minDuration = motionProfile.getMinDuration();

    }
}
