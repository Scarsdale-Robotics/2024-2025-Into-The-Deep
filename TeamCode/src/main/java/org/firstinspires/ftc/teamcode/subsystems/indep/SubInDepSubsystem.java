package org.firstinspires.ftc.teamcode.subsystems.indep;

public abstract class SubInDepSubsystem<State> {

    protected boolean approxEq(double a, double b) {
        double epsilon = 0.000001;
        return Math.abs(a-b) <= epsilon;
    }

    public abstract void setState(State state);

    public abstract State getState();

    public abstract boolean jobFulfilled();

}
