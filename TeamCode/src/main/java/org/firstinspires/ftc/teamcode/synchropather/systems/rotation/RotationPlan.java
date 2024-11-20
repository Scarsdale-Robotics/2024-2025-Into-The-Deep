package org.firstinspires.ftc.teamcode.synchropather.systems.rotation;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.RobotSystem;
import org.firstinspires.ftc.teamcode.synchropather.systems.MovementType;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.Movement;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.Plan;

import java.util.ArrayList;

/**
 * Object containing a sequence of Movements for rotational drive.
 */
@Config
public class RotationPlan extends Plan<RotationState> {

	// Feedforward constants
	//TODO: TUNE
	public static double kS = 0;
	public static double kV = 1;
	public static double kA = 0.1;

	// PD constants
	//TODO: TUNE
	public static double kP = 4;
	public static double kD = 0;

	// Error history array
	private final ArrayList<Double> eHistory;

	private RobotSystem robot;

	/**
	 * Creates a new RotationPlan object with the given Movements.
	 * @param robot The RobotSystem containing a DriveSubsystem.
	 * @param movements
	 */
	public RotationPlan(RobotSystem robot, Movement... movements) {
		super(MovementType.ROTATION, movements);
		this.robot = robot;
		this.eHistory = new ArrayList<>();
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
		double e = normalizeAngle(error.getHeading());

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
		u += kS*Math.signum(dv) + kV*dv + kA*da;

		// Set drive powers
		robot.drive.turnVelocity = u;
		robot.drive.driveFieldCentric(currentPose.getHeading()); // note: function overload
		robot.drive.targetH = desiredState.getHeading();

		robot.telemetry.addData("[SYNCHROPATHER] Rotation desiredVelocity.getHeading()", desiredVelocity.getHeading());
		robot.telemetry.addData("[SYNCHROPATHER] Rotation desiredState.getHeading()", desiredState.getHeading());
		robot.telemetry.addData("[SYNCHROPATHER] Rotation error.getHeading()", error.getHeading());
		robot.telemetry.addData("[SYNCHROPATHER] robot.drive.turnVelocity", robot.drive.turnVelocity);
	}

	@Override
	public void stop() {
		robot.drive.turnVelocity = 0;
		robot.drive.stopController();
	}

	/**
	 * Normalizes a given angle to (-pi,pi] radians.
	 * @param radians the given angle in radians.
	 * @return the normalized angle in radians.
	 */
	private static double normalizeAngle(double radians) {
		while (radians >= Math.PI) radians -= 2*Math.PI;
		while (radians < -Math.PI) radians += 2*Math.PI;
		return radians;
	}

}
