package org.firstinspires.ftc.teamcode.synchropather.systems.__util__.motion_profiles;

import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.TimeSpan;

/**
 * Calculates position based on elapsed time from a velocity curve defined by displacement, initial velocity, max velocity, max acceleration.
 */
public class DynamicMotionProfile1D extends MotionProfile1D {

    private final double distance, sign;

    private final double v_max, a_max;
    private double t1, t2, t3, t4, T;
    private double v0, v1, v2;
    private double D1, D2, D3, D4;

    private TimeSpan timeSpan;

    /**
     * Creates a new DynamicMotionProfile1D object with the given parameters.
     * @param targetDisplacement units
     * @param startTime seconds
     * @param v0 units/second
     * @param v_max units/second
     * @param a_max units/second^2
     */
    public DynamicMotionProfile1D(double targetDisplacement, double startTime, double v0, double v_max, double a_max) {
        this.sign = Math.signum(targetDisplacement);
        this.distance = Math.abs(targetDisplacement);
        this.v0 = v0;
        this.v_max = v_max;
        this.a_max = a_max;
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
            displacement = v0*t + 0.5*a_max*tt;
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
            displacement = D2 + v1*t - 0.5*a_max*tt;
        }
        // Accelerating correction
        else if (elapsedTime <= t4) {
            double t = elapsedTime-t3;
            double tt = t*t;
            displacement = D3 - 0.5*a_max*tt;
        }
        // Decelerating correction
        else {
            double t = elapsedTime-t4;
            double tt = t*t;
            displacement = D4 + v2*t + 0.5*a_max*tt;
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
            velocity = v0 + a_max*t;
        }
        // Cruising
        else if (elapsedTime <= t2) {
            velocity = v1;
        }
        // Decelerating
        else if (elapsedTime <= t3) {
            double t = elapsedTime-t2;
            velocity = v1 - a_max*t;
        }
        // Accelerating correction
        else if (elapsedTime <= t4) {
            double t = elapsedTime-t3;
            velocity = -a_max*t;
        }
        // Decelerating correction
        else {
            double t = elapsedTime-t4;
            velocity = v2 + a_max*t;
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
            acceleration = a_max;
        }
        // Cruising
        else if (elapsedTime <= t2) {
            acceleration = 0;
        }
        // Decelerating
        else if (elapsedTime <= t3) {
            acceleration = -a_max;
        }
        // Accelerating correction
        else if (elapsedTime <= t4) {
            acceleration = -a_max;
        }
        // Decelerating correction
        else {
            acceleration = a_max;
        }

        acceleration *= sign;

        return acceleration;
    }

    private void init() {
        //// Breaking the problem up into cases
        double D3 = v0*v0 / (2*a_max);
        double D1plusD3_max = (2*v_max*v_max - v0*v0) / (2*a_max);
        double D2 = distance - (D1plusD3_max);
        double D4 = Math.max(0,D3 - distance);

        // Case 1
        // Right triangle
        if (distance == D3) {
            // velocities
            v1 = v0;
            v2 = 0;
            // times
            t1 = 0;
            t2 = 0;
            t3 = v0/a_max;
            t4 = t3;
            T = t4;
            // distances
            this.D1 = 0;
            this.D2 = this.D1 + 0;
            this.D3 = this.D2 + v0*v0/(2*a_max);
            this.D4 = this.D3 + 0;
        }

        // Case 2
        // Truncated isosceles triangle
        else if (distance > D3 && distance <= D1plusD3_max) {
            // velocities
            v1 = Math.sqrt(distance*a_max + v0*v0/2);
            v2 = 0;
            // times
            t1 = (v1-v0)/a_max;
            t2 = t1;
            t3 = t1 + (v1/a_max);
            t4 = t3;
            T = t4;
            // distances
            this.D1 = (v1*v1-v0*v0)/(2*a_max);
            this.D2 = this.D1 + 0;
            this.D3 = this.D2 + v1*v1/(2*a_max);
            this.D4 = this.D3 + 0;
        }

        // Case 3
        // Truncated trapezoid
        else if (distance > D1plusD3_max) {
            // velocities
            v1 = v_max;
            v2 = 0;
            // times
            t1 = (v_max-v0)/a_max;
            t2 = t1 + (D2/v_max);
            t3 = t2 + (v_max/a_max);
            t4 = t3;
            T = t4;
            // distances
            this.D1 = (v_max*v_max-v0*v0)/(2*a_max);
            this.D2 = this.D1 + D2;
            this.D3 = this.D2 + (v_max*v_max)/(2*a_max);
            this.D4 = this.D3 + 0;
        }

        // Case 4
        // Overshot right triangle
        else {
            // velocities
            v1 = v0;
            v2 = -Math.sqrt(D4*a_max);
            // times
            t1 = 0;
            t2 = 0;
            t3 = v0/a_max;
            t4 = t3 + Math.sqrt(D4/a_max);
            T = 2*t4 - t3;
            // distances
            this.D1 = 0;
            this.D2 = this.D1 + 0;
            this.D3 = this.D2 + (v0*v0)/(2*a_max);
            this.D4 = this.D3 + D4/2;
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
