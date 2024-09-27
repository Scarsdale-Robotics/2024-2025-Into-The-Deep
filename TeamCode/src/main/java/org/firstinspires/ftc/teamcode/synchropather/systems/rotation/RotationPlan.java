package org.firstinspires.ftc.teamcode.synchropather.systems.rotation;

import com.arcrobotics.ftclib.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.RobotSystem;
import org.firstinspires.ftc.teamcode.synchropather.systems.MovementType;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.Movement;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.Plan;

import java.util.ArrayList;

/**
 * Object containing a sequence of Movements for rotational drive.
 */
public class RotationPlan extends Plan<RotationState> {

	// Feedforward constants
	//TODO: TUNE
	private static final double kS = 0;
	private static final double kV = 1;
	private static final double kA = 0;

	// PD constants
	//TODO: TUNE
	private static final double kP = 1;
	private static final double kD = 0;

	// Error history array
	private ArrayList<Double> eHistory;

	private RobotSystem robot;

	/**
	 * Creates a new RotationPlan object with the given Movements.
	 * @param robot The RobotSystem containing a DriveSubsystem.
	 * @param movements
	 */
	public RotationPlan(RobotSystem robot, Movement... movements) {
		super(MovementType.ROTATION, movements);
		this.robot = robot;
	}

	/**
	 * Controls the rotation output of the robot to the RotationState at the elapsedTime.
	 */
	@Override
	public void loop() {
		// Desired states
		RotationState desiredState = getCurrentState();
		RotationState desiredVelocity = getCurrentVelocity();
		RotationState desiredAcceleration = getCurrentAcceleration();
		double dv = desiredVelocity.getHeading();
		double da = desiredAcceleration.getHeading();

		// Current state
		Pose2d currentPose = robot.localization.getPose();
		RotationState currentState = new RotationState(currentPose.getHeading());

		// State error
		RotationState error = desiredState.minus(currentState);
		double e = error.getHeading();

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
		robot.drive.turnVelocity = u;
		robot.drive.driveFieldCentric(currentPose.getHeading());
	}

}
