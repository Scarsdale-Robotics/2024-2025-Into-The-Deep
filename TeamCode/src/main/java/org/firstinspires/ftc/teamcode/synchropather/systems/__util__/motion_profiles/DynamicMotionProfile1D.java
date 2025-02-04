package org.firstinspires.ftc.teamcode.synchropather.systems.__util__.motion_profiles;

import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.TimeSpan;

/**
 * Calculates position based on elapsed time from a velocity curve defined by displacement, initial velocity, max velocity, max acceleration, and max deceleration.
 */
public class DynamicMotionProfile1D extends MotionProfile1D {

    private final double distance, sign;

    private final double v_max, a_max_1, a_max_2;
    private double t1, t2, t3, t4, T;
    private final double v0;
    private double v1, v2;
    private double D1, D2, D3, D4;

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
    public DynamicMotionProfile1D(double targetDisplacement, double startTime, double v0, double v_max, double a_max_1, double a_max_2) {
        this.sign = Math.signum(targetDisplacement);
        this.distance = Math.abs(targetDisplacement);
        this.v_max = Math.abs(v_max);
        this.a_max_1 = Math.abs(a_max_1);
        this.a_max_2 = Math.abs(a_max_2);
        this.v0 = bound(v0 * this.sign, -this.v_max, this.v_max);
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
            displacement = v0*t + 0.5*a_max_1*tt;
        }
        // Cruising
        else if (elapsedTime <= t2) {
            double t = elapsedTime-t1;
            displacement = D1 + v1*t;
        }
        // Decelerating
        else if (elapsedTime <= t3) {
            double t = elapsedTime-t2;
            double tt = t*t;
            displacement = D2 + v1*t - 0.5*a_max_2*tt;
        }
        // Accelerating correction
        else if (elapsedTime <= t4) {
            double t = elapsedTime-t3;
            double tt = t*t;
            displacement = D3 - 0.5*a_max_2*tt;
        }
        // Decelerating correction
        else {
            double t = elapsedTime-t4;
            double tt = t*t;
            displacement = D4 + v2*t + 0.5*a_max_2*tt;
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
            velocity = v0 + a_max_1*t;
        }
        // Cruising
        else if (elapsedTime <= t2) {
            velocity = v1;
        }
        // Decelerating
        else if (elapsedTime <= t3) {
            double t = elapsedTime-t2;
            velocity = v1 - a_max_2*t;
        }
        // Accelerating correction
        else if (elapsedTime <= t4) {
            double t = elapsedTime-t3;
            velocity = -a_max_2*t;
        }
        // Decelerating correction
        else {
            double t = elapsedTime-t4;
            velocity = v2 + a_max_2*t;
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
        else if (elapsedTime <= t3) {
            acceleration = -a_max_2;
        }
        // Accelerating correction
        else if (elapsedTime <= t4) {
            acceleration = -a_max_2;
        }
        // Decelerating correction
        else {
            acceleration = a_max_2;
        }

        acceleration *= sign;

        return acceleration;
    }

    private void init() {
        //// Breaking the problem up into cases
        double D1plusD3_max = v_max*v_max*(a_max_1+a_max_2)/(2*a_max_1*a_max_2) - v0*v0/(2*a_max_1);
        double D3_min = v0*v0/(2*a_max_2);

        // Case 1
        // Right triangle
        if (distance == D3_min) {
            // velocities
            v1 = v0;
            v2 = 0;
            // times
            t1 = 0;
            t2 = 0;
            t3 = v0/a_max_2;
            t4 = t3;
            T = t4;
            // distances
            this.D1 = 0;
            this.D2 = this.D1 + 0;
            this.D3 = this.D2 + v0*v0/(2*a_max_2);
            this.D4 = this.D3 + 0;
        }

        // Case 2
        // Truncated isosceles triangle
        else if ((distance > D3_min || v0 < 0) && distance <= D1plusD3_max) {
            // velocities
            v1 = Math.sqrt( (distance + v0*v0/(2*a_max_1)) * 2*a_max_1*a_max_2/(a_max_1+a_max_2) );
            v2 = 0;
            // times
            t1 = (v1-v0)/a_max_1;
            t2 = t1;
            t3 = t1 + (v1/a_max_2);
            t4 = t3;
            T = t4;
            // distances
            this.D1 = (v1*v1-v0*v0)/(2*a_max_1);
            this.D2 = this.D1 + 0;
            this.D3 = this.D2 + v1*v1/(2*a_max_2);
            this.D4 = this.D3 + 0;
        }

        // Case 3
        // Truncated trapezoid
        else if (distance > D1plusD3_max || v0 < 0) {
            // distances
            this.D1 = (v_max*v_max-v0*v0)/(2*a_max_1);
            this.D2 = this.D1 + distance - D1plusD3_max;
            this.D3 = this.D2 + (v_max*v_max)/(2*a_max_2);
            this.D4 = this.D3 + 0;
            // velocities
            v1 = v_max;
            v2 = 0;
            // times
            t1 = (v_max-v0)/a_max_1;
            t2 = t1 + (this.D2-this.D1)/v_max;
            t3 = t2 + (v_max/a_max_2);
            t4 = t3;
            T = t4;
        }

        // Case 4
        // Overshot right triangle
        else {
            // distances
            this.D1 = 0;
            this.D2 = this.D1 + 0;
            this.D3 = this.D2 + (v0*v0)/(2*a_max_2);
            this.D4 = this.D3 + (this.D3-distance)/2;
            // velocities
            v1 = v0;
            v2 = -Math.sqrt((this.D3-distance)*a_max_2);
            // times
            t1 = 0;
            t2 = t1 + 0;
            t3 = t2 + v0/a_max_2;
            t4 = t3 + Math.sqrt((this.D3-distance)/a_max_2);
            T = 2*t4 - t3;
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
