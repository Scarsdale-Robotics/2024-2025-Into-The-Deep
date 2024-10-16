package org.firstinspires.ftc.teamcode.synchropather.systems.lift;

import com.arcrobotics.ftclib.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.RobotSystem;
import org.firstinspires.ftc.teamcode.synchropather.systems.MovementType;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.Movement;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.Plan;
import org.firstinspires.ftc.teamcode.synchropather.systems.rotation.RotationState;

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

    private final ArrayList<Double> eHistory;
    private RobotSystem robot;

    public LiftPlan(RobotSystem robot, Movement... movements) {
        super(MovementType.LIFT, movements);
        this.robot = robot;
        this.eHistory = new ArrayList<>();
    }

    public void loop() {
        // Desired states
        LiftState desiredState = getCurrentState();
        LiftState desiredVelocity = getCurrentVelocity();
        LiftState desiredAcceleration = getCurrentAcceleration();
        double dv = desiredVelocity.getHeight();
        double da = desiredAcceleration.getHeight();

        // Current state
        double currentPosition = robot.inDep.getLiftPosition();
        // lift subsystem get height here instead
        LiftState currentState = new LiftState(currentPosition); // motor position --> height idk

        // State error
        LiftState error = desiredState.minus(currentState);
        double e = error.getHeight();

        // Error derivatives
        double dedt = 0;
        eHistory.add(e);
        if (eHistory.size()>5) eHistory.remove(0);
        if (eHistory.size()==5) dedt = robot.localization.stencil(eHistory); // this is pos2D but still works (same formula)

        // Control output
        double u = 0;

        // Lift PD
        u += kP*e + kD*dedt;

        // Feedforward
        if (error.abs() < Math.PI/18) {
            u += kS*Math.signum(dv) + kV*dv + kA*da;
        }

        // Set drive powers
//        robot.drive.turnVelocity = u;
//        robot.drive.driveFieldCentric(currentPose.getHeading());
        robot.inDep.setLiftPower(u);
    }

    @Override
    public void stop() {
        robot.inDep.setLiftPower(0);
    }
}
