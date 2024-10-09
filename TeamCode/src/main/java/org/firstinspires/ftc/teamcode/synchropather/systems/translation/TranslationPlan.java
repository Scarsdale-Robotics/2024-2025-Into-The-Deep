package org.firstinspires.ftc.teamcode.synchropather.systems.translation;

import com.arcrobotics.ftclib.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.RobotSystem;
import org.firstinspires.ftc.teamcode.synchropather.systems.MovementType;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.Movement;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.Plan;

import java.util.ArrayList;

/**
 * Object containing a sequence of Movements for translational drive.
 */
public class TranslationPlan extends Plan<TranslationState> {

	// Feedforward constants
	//TODO: TUNE
	private static final double kS = 0;
	private static final double kV = 1;
	private static final double kA = 0;

	// Positional PD constants
	//TODO: TUNE
	private static final double kP = 1;
	private static final double kD = 0;

	// Error history array
	private ArrayList<Double> exHistory;
	private ArrayList<Double> eyHistory;

	private RobotSystem robot;

	/**
	 * Creates a new TranslationPlan object with the given Movements.
	 * @param robot The RobotSystem containing a DriveSubsystem.
	 * @param movements
	 */
	public TranslationPlan(RobotSystem robot, Movement... movements) {
		super(MovementType.TRANSLATION, movements);
		this.robot = robot;
	}

	/**
	 * Controls the translation output of the robot to the TranslationState at the elapsedTime.
	 */
	@Override
	public void loop() {
		// Desired states
		TranslationState desiredState = getCurrentState();
		TranslationState desiredVelocity = getCurrentVelocity();
		TranslationState desiredAcceleration = getCurrentAcceleration();
		double dxv = desiredVelocity.getX();
		double dyv = desiredVelocity.getY();
		double dxa = desiredAcceleration.getX();
		double dya = desiredAcceleration.getY();

		// Current state
		Pose2d currentPose = robot.localization.getPose();
		TranslationState currentState = new TranslationState(currentPose.getX(), currentPose.getY());

		// State error
		TranslationState error = desiredState.minus(currentState);
		double ex = error.getX();
		double ey = error.getY();

		// Error derivatives
		double dexdt = 0;
		double deydt = 0;
		exHistory.add(ex);
		eyHistory.add(ey);
		if (exHistory.size()>5) exHistory.remove(0);
		if (eyHistory.size()>5) eyHistory.remove(0);
		if (exHistory.size()==5) dexdt = robot.localization.stencil(exHistory);
		if (eyHistory.size()==5) deydt = robot.localization.stencil(eyHistory);

		// Control output
		double ux = 0;
		double uy = 0;

		// Positional PD
		ux += kP*ex + kD*dexdt;
		uy += kP*ey + kD*deydt;

		// Feedforward
		if (error.hypot() < 6) {
			ux += kS * Math.signum(dxv) + kV * dxv + kA * dxa;
			uy += kS * Math.signum(dyv) + kV * dyv + kA * dya;
		}

		// Set drive parameters
		TranslationState ut = new TranslationState(ux, uy);
		robot.drive.driveTheta = ut.theta();
		robot.drive.driveSpeed = ut.hypot();
		robot.drive.driveFieldCentric(currentPose.getHeading());
	}

}
