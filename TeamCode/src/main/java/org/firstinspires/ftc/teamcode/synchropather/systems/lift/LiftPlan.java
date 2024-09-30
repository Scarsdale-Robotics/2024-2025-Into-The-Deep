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

    private ArrayList<Double> eHistory;
    private RobotSystem robot;
    public LiftPlan(RobotSystem robot, Movement... movements) {
        super(MovementType.LIFT, movements);
        this.robot = robot;
    }

    public void loop() {
        // Desired states
        LiftState desiredState = getCurrentState();
        LiftState desiredVelocity = getCurrentVelocity();
        LiftState desiredAcceleration = getCurrentAcceleration();
        double dv = desiredVelocity.getHeight();
        double da = desiredAcceleration.getHeight();

        // Current state
        Pose2d currentPose = robot.localization.getPose();
        lift subsystem get height here instead
        LiftState currentState = new LiftState(currentPose.getHeight());

        // State error
        LiftState error = desiredState.minus(currentState);
        double e = error.getHeight();

        // Error derivatives
        double dedt = 0;
        eHistory.add(e);
        if (eHistory.size()>5) eHistory.remove(0);
        if (eHistory.size()==5) dedt = robot.localization.stencil(eHistory);

        // Control output
        double u = 0;

        // Rotational PD
        u += kP*e + kD*dedt;

        // Feedforward
        if (error.abs() < Math.PI/18) {
            u += kS*Math.signum(dv) + kV*dv + kA*da;
        }

        // Set drive powers
//        robot.drive.turnVelocity = u;
//        robot.drive.driveFieldCentric(currentPose.getHeading());
        add motor stuff here
    }
}
