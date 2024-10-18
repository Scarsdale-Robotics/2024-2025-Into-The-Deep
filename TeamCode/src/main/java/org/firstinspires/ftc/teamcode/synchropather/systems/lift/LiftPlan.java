package org.firstinspires.ftc.teamcode.synchropather.systems.lift;

import org.firstinspires.ftc.teamcode.RobotSystem;
import org.firstinspires.ftc.teamcode.synchropather.systems.MovementType;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.Movement;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.Plan;

import java.util.ArrayList;

public class LiftPlan extends Plan<LiftState> {
    // Feedforward constants
    //TODO: TUNE
    private static final double kS = 0;
    private static final double kV = 1;
    private static final double kA = 0;

    // Positional PD constants
    //TODO: TUNE
    private static final double kP = 1;
    private static final double kD = 0;

    private final ArrayList<Double> leHistory;
    private final ArrayList<Double> reHistory;
    private RobotSystem robot;

    public LiftPlan(RobotSystem robot, Movement... movements) {
        super(MovementType.LIFT, movements);
        this.robot = robot;
        this.leHistory = new ArrayList<>();
        this.reHistory = new ArrayList<>();
    }

    public void loop() {
        // Desired states
        LiftState desiredState = getCurrentState();
        LiftState desiredVelocity = getCurrentVelocity();
        LiftState desiredAcceleration = getCurrentAcceleration();
        double dv = desiredVelocity.getHeight();
        double da = desiredAcceleration.getHeight();

        // Current state
        double leftHeight = robot.inDep.getLeftLiftPosition();
        double rightHeight = robot.inDep.getRightLiftPosition();
        // lift subsystem get height here instead
        LiftState currentLeftState = new LiftState(leftHeight); // motor position --> height idk
        LiftState currentRightState = new LiftState(rightHeight); // motor position --> height idk

        // State error
        LiftState leftError = desiredState.minus(currentLeftState);
        LiftState rightError = desiredState.minus(currentRightState);
        double le = leftError.getHeight();
        double re = rightError.getHeight();

        // Error derivatives
        double ldedt = 0;
        double rdedt = 0;
        leHistory.add(le);
        reHistory.add(re);
        if (leHistory.size()>5) leHistory.remove(0);
        if (reHistory.size()>5) reHistory.remove(0);
        if (leHistory.size()==5) ldedt = robot.localization.stencil(leHistory); // this is pos2D but still works (same formula)
        if (reHistory.size()==5) rdedt = robot.localization.stencil(reHistory); // this is pos2D but still works (same formula)

        // Control output
        double lu = 0;
        double ru = 0;

        // Lift PD
        lu += kP*le + kD*ldedt;
        ru += kP*re + kD*rdedt;

        // Feedforward
        double fu = kS*Math.signum(dv) + kV*dv + kA*da;
        lu += fu;
        ru += fu;

        // Set drive powers
        robot.inDep.setLeftLiftPower(lu);
        robot.inDep.setRightLiftPower(ru);
    }

    @Override
    public void stop() {
        robot.inDep.setLeftLiftPower(0);
        robot.inDep.setRightLiftPower(0);
    }
}
