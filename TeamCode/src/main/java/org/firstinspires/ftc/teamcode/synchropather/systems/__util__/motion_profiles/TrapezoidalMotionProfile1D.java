package org.firstinspires.ftc.teamcode.synchropather.systems.__util__.motion_profiles;

import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.TimeSpan;

/**
 * Calculates position based on elapsed time from a velocity curve defined by displacement, max velocity, max acceleration, and max deceleration.
 */
public class TrapezoidalMotionProfile1D extends MotionProfile1D {

    private final double distance, sign;

    private final double v_max, a_max_1, a_max_2;
    private double t1, t2, T;
    private double v1;
    private double D1, D2;

    private TimeSpan timeSpan;

    /**
     * Creates a new DynamicMotionProfile1D object with the given parameters.
     * @param targetDisplacement units
     * @param startTime seconds
     * @param v0 units/second
     * @param v_max units/second
     * @param a_max_1 ACCELERATION units/second^2
     * @param a_max_2 DECELERATION units/second^2
     */
    public TrapezoidalMotionProfile1D(double targetDisplacement, double startTime, double v0, double v_max, double a_max_1, double a_max_2) {
        this.sign = Math.signum(targetDisplacement);
        this.distance = Math.abs(targetDisplacement);
        this.v_max = v_max;
        this.a_max_1 = a_max_1;
        this.a_max_2 = a_max_2;
        init();
        this.timeSpan = new TimeSpan(startTime, startTime+T);
    }

    /**
     * @return the timestamp for when the Movement starts.
     */
    public double getStartTime() {
        return timeSpan.getStartTime();
    }

    /**
     * @return the timestamp for when the Movement ends.
     */
    public double getEndTime() {
        return timeSpan.getEndTime();
    }

    /**
     * @return the time needed to reach the target displacement value in seconds
     */
    @Override
    public double getDuration() {
        return timeSpan.getDuration();
    }

    @Override
    public double getDisplacement(double elapsedTime) {
        elapsedTime = bound(elapsedTime-getStartTime(), 0, getDuration());


        double displacement;

        // Accelerating
        if (elapsedTime <= t1) {
            double t = elapsedTime-0;
            double tt = t*t;
            displacement = 0.5*a_max_1*tt;
        }
        // Cruising
        else if (elapsedTime <= t2) {
            double t = elapsedTime-t1;
            displacement = D1 + v1*t;
        }
        // Decelerating
        else {
            double t = elapsedTime-t2;
            double tt = t*t;
            displacement = D2 + v1*t - 0.5*a_max_2*tt;
        }

        displacement *= sign;

        return displacement;
    }

    @Override
    public double getVelocity(double elapsedTime) {
        if (distance == 0) return 0;
        elapsedTime = bound(elapsedTime-getStartTime(), 0, getDuration());

        double velocity;

        // Accelerating
        if (elapsedTime <= t1) {
            double t = elapsedTime-0;
            velocity = a_max_1*t;
        }
        // Cruising
        else if (elapsedTime <= t2) {
            velocity = v1;
        }
        // Decelerating
        else {
            double t = elapsedTime-t2;
            velocity = v1 - a_max_2*t;
        }

        velocity *= sign;

        return velocity;
    }

    @Override
    public double getAcceleration(double elapsedTime) {
        if (distance == 0) return 0;
        // Zero acceleration if outside of TimeSpan.
        if (elapsedTime-getStartTime() < 0 || getDuration() < elapsedTime-getStartTime()) return 0;

        elapsedTime = bound(elapsedTime-getStartTime(), 0, getDuration());

        double acceleration;

        // Accelerating
        if (elapsedTime <= t1) {
            acceleration = a_max_1;
        }
        // Cruising
        else if (elapsedTime <= t2) {
            acceleration = 0;
        }
        // Decelerating
        else {
            acceleration = -a_max_2;
        }

        acceleration *= sign;

        return acceleration;
    }

    private void init() {
        //// Breaking the problem up into cases
        double D1plusD3_max = v_max*v_max*(a_max_1+a_max_2) / (2*a_max_1*a_max_2);

        // Case 1
        // Trapezoid
        if (distance > D1plusD3_max) {
            // distances
            this.D1 = v_max*v_max/(2*a_max_1);
            this.D2 = this.D1 + distance - D1plusD3_max;
            // velocities
            v1 = v_max;
            // times
            t1 = v_max/a_max_1;
            t2 = t1 + (D2-D1)/v_max;
            T = t2 + v_max/a_max_2;
        }

        // Case 2
        // Triangle
        else {
            // velocities
            v1 = Math.sqrt(2*a_max_1*a_max_2*distance/(a_max_1+a_max_2));
            // times
            t1 = v1/a_max_1;
            t2 = t1;
            T = t2 + v1/a_max_2;
            // distances
            this.D1 = v1*v1/(2*a_max_1);
            this.D2 = this.D1;
        }
    }

    /**
     * Clips the input x between a given lower and upper bound.
     * @param x
     * @param lower
     * @param upper
     * @return the clipped value of x.
     */
    private static double bound(double x, double lower, double upper) {
        return Math.max(lower, Math.min(upper, x));
    }

}
