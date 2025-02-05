package org.firstinspires.ftc.teamcode.synchropather.systems.__util__.motion_profiles;

import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.TimeSpan;

/**
 * Calculates position based on elapsed time from a velocity curve defined by displacement, max velocity, max acceleration, and max deceleration.
 */
public class TrapezoidalMotionProfile1D extends MotionProfile1D {

    private final DynamicMotionProfile1D dynamicMotionProfile;

    /**
     * Creates a new TrapezoidalMotionProfile1D object with the given parameters.
     * @param targetDisplacement units
     * @param startTime seconds
     * @param v_max units/second
     * @param a_max_1 ACCELERATION units/second^2
     * @param a_max_2 DECELERATION units/second^2
     */
    public TrapezoidalMotionProfile1D(double targetDisplacement, double startTime, double v_max, double a_max_1, double a_max_2) {
        this.dynamicMotionProfile = new DynamicMotionProfile1D(
                targetDisplacement,
                startTime,
                0,
                v_max,
                a_max_1,
                a_max_2
        );
    }

    /**
     * Creates a new TrapezoidalMotionProfile1D object with the given parameters.
     * @param targetDisplacement units
     * @param timeSpan range of seconds
     * @param v_max units/second
     * @param a_max_1 ACCELERATION units/second^2
     * @param a_max_2 DECELERATION units/second^2
     */
    public TrapezoidalMotionProfile1D(double targetDisplacement, TimeSpan timeSpan, double v_max, double a_max_1, double a_max_2) {
        this.dynamicMotionProfile = new DynamicMotionProfile1D(
                targetDisplacement,
                timeSpan.getStartTime(),
                0,
                v_max,
                a_max_1,
                a_max_2
        );
        this.dynamicMotionProfile.setTimeSpan(timeSpan);
    }

    /**
     * @return the TimeSpan of this motion profile.
     */
    public TimeSpan getTimeSpan() {
        return dynamicMotionProfile.getTimeSpan();
    }

    /**
     * @return the timestamp for when the Movement starts.
     */
    public double getStartTime() {
        return dynamicMotionProfile.getStartTime();
    }

    /**
     * @return the timestamp for when the Movement ends.
     */
    public double getEndTime() {
        return dynamicMotionProfile.getEndTime();
    }

    /**
     * @return the time needed to reach the target displacement value in seconds
     */
    @Override
    public double getDuration() {
        return dynamicMotionProfile.getDuration();
    }

    /**
     * @return the minimum time needed to reach the target displacement value.
     */
    public double getMinDuration() {
        return dynamicMotionProfile.getMinDuration();
    }

    @Override
    public double getDisplacement(double elapsedTime) {
        return dynamicMotionProfile.getDisplacement(elapsedTime);
    }

    @Override
    public double getVelocity(double elapsedTime) {
        return dynamicMotionProfile.getVelocity(elapsedTime);
    }

    @Override
    public double getAcceleration(double elapsedTime) {
        return dynamicMotionProfile.getAcceleration(elapsedTime);
    }

}
