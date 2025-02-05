package org.firstinspires.ftc.teamcode.synchropather.systems.__util__.motion_profiles;

import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.TimeSpan;

/**
 * Calculates position based on elapsed time from a velocity curve defined by displacement, initial velocity, max velocity, max acceleration, and max deceleration.
 */
public class DynamicMotionProfile1D extends MotionProfile1D {

    private double distance, sign, minDuration;

    private double v_max, a_max_1, a_max_2;
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
        this.sign = targetDisplacement != 0 ? Math.signum(targetDisplacement) : 1;
        this.distance = Math.abs(targetDisplacement);
        this.v_max = Math.abs(v_max);
        this.a_max_1 = Math.abs(a_max_1);
        this.a_max_2 = Math.abs(a_max_2);
        this.v0 = bound(v0 * (this.sign!=0 ? this.sign : 1), -this.v_max, this.v_max);
        init();
        this.timeSpan = new TimeSpan(startTime, startTime+T);
        this.minDuration = this.timeSpan.getDuration();
    }

    /**
     * Creates a new DynamicMotionProfile1D object with the given parameters.
     * @param targetDisplacement units
     * @param timeSpan in seconds
     * @param v0 units/second
     * @param v_max units/second
     * @param a_max_1 ACCELERATION units/second^2
     * @param a_max_2 DECELERATION units/second^2
     */
    public DynamicMotionProfile1D(double targetDisplacement, TimeSpan timeSpan, double v0, double v_max, double a_max_1, double a_max_2) {
        this(targetDisplacement, timeSpan.getStartTime(), v0, v_max, a_max_1, a_max_2);
        setTimeSpan(timeSpan);
    }

    /**
     * @return the TimeSpan of this motion profile.
     */
    public TimeSpan getTimeSpan() {
        return timeSpan;
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
    public double getDuration() {
        return timeSpan.getDuration();
    }

    /**
     * @return the minimum time needed to reach the target displacement value.
     */
    public double getMinDuration() {
        return minDuration;
    }

    /**
     * @param newTimeSpan another timeSpan with a duration greater than minDuration
     */
    public void setTimeSpan(TimeSpan newTimeSpan) {
        /// catch error for when time < min_time
        if (newTimeSpan.getDuration() - minDuration < -1e-3) {
            throw new RuntimeException(
                    String.format("TimeSpan duration %s is less than the minimum needed time %s.",
                            newTimeSpan.getDuration(),
                            minDuration
                    )
            );
        }

        // if the new timeSpan has the same duration as minDuration
        if (newTimeSpan.getDuration() < minDuration) {
            this.timeSpan = new TimeSpan(newTimeSpan.getStartTime(), newTimeSpan.getStartTime() + minDuration);
            return;
        }

        // if the curve is unstretchable
        if (v0>0 && distance <= v0*v0/(2*a_max_1)) {
            this.timeSpan = newTimeSpan;
            return;
        }


        //// Two cases based on tau, which is when v0=v1 (_p means prime)
        double tau = v0/(2*a_max_2);
        if (v0 != 0) tau += distance/v0 + v2*v2/(v0*a_max_2);
        else tau = Double.POSITIVE_INFINITY;
        double T_p = newTimeSpan.getDuration();

        // Case 1: Use quadratic formula
        double t1_p, t2_p, t3_p, t4_p;
        double D1_p, D2_p, D3_p, D4_p;
        double v1_p;
        double a_max_1_p;
        if (v0<=0 || (v0>0 && T_p<=tau)) {
            double a = (a_max_1+a_max_2)/(2*a_max_1*a_max_2);
            double b = -(T_p + v0/a_max_1);
            double c = distance + v0*v0/(2*a_max_1);
            // velocities
            v1_p = (-b-Math.sqrt(b*b-4*a*c)) / (2*a);
            // times
            t1_p = (v1_p-v0)/a_max_1;
            t2_p = T_p - (T-t3) - v1_p/a_max_2;
            t3_p = t2_p + v1_p/a_max_2;
            t4_p = t3_p + (t4-t3);
            // distances
            D1_p = (v1_p*v1_p-v0*v0)/(2*a_max_1);
            D2_p = distance + v2*v2/a_max_2 - v1_p*v1_p/(2*a_max_2);
            D3_p = D2_p + v1_p*v1_p/(2*a_max_2);
            D4_p = D3_p - v2*v2/(2*a_max_2);
            // accelerations
            a_max_1_p = a_max_1;
        }

        // Case 2: when 0<v0<v1
        else {
            // velocities
            v1_p = (2*distance*a_max_2-v0*v0+2*v2*v2) / (2*T_p*a_max_2-2*v0);
            // times
            t1_p = (v0-v1_p)/a_max_2;
            t2_p = t1_p + T_p - v0/a_max_2;
            t3_p = t2_p + v1_p/a_max_2;
            t4_p = t3_p + (T-t4);
            // distances
            D1_p = (v0*v0-v1_p*v1_p)/(2*a_max_2);
            D2_p = distance + v2*v2/a_max_2 - v1_p*v1_p/(2*a_max_2);
            D3_p = D2_p + v1_p*v1_p/(2*a_max_2);
            D4_p = D3_p - v2*v2/(2*a_max_2);
            // accelerations
            a_max_1_p = -a_max_2;
        }

        //// replace current values
        // velocities
        this.v1 = v1_p;
        // times
        this.t1 = t1_p;
        this.t2 = t2_p;
        this.t3 = t3_p;
        this.t4 = t4_p;
        this.T = T_p;
        // distances
        this.D1 = D1_p;
        this.D2 = D2_p;
        this.D3 = D3_p;
        this.D4 = D4_p;
        // accelerations
        this.a_max_1 = a_max_1_p;
        // timeSpan
        this.timeSpan = newTimeSpan;

    }

    public double getDisplacement(double elapsedTime) {
        // Zero displacement if T=0.
        if (T==0) return 0;

        elapsedTime = bound(elapsedTime-getStartTime(), 0, T);

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

    public double getVelocity(double elapsedTime) {
        // Zero velocity if T=0.
        if (T==0) return 0;
        // Zero velocity if outside of TimeSpan.
        if (elapsedTime-getStartTime() < 0 || T < elapsedTime-getStartTime()) return 0;

        elapsedTime = bound(elapsedTime-getStartTime(), 0, T);

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

    public double getAcceleration(double elapsedTime) {
        // Zero acceleration if T=0.
        if (T==0) return 0;
        // Zero acceleration if outside of TimeSpan.
        if (elapsedTime-getStartTime() < 0 || T < elapsedTime-getStartTime()) return 0;

        elapsedTime = bound(elapsedTime-getStartTime(), 0, T);

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
        if (distance == D3_min && v0 >= 0) {
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
            this.D4 = this.D3 - (this.D3-distance)/2;
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
