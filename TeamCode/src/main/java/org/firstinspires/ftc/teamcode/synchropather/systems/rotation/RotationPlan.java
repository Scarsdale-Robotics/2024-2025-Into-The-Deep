package org.firstinspires.ftc.teamcode.synchropather.systems.rotation;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotSystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LocalizationSubsystem;
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

	private DriveSubsystem drive;
	private LocalizationSubsystem localization;
	private Telemetry telemetry;

	/**
	 * Creates a new RotationPlan object with the given Movements.
	 * @param drive the robot's DriveSubsystem.
	 * @param localization the robot's LocalizationSubsystem.
	 * @param telemetry the robot's telemetry
	 * @param movements
	 */
	public RotationPlan(DriveSubsystem drive, LocalizationSubsystem localization, Telemetry telemetry, Movement... movements) {
		super(MovementType.ROTATION, movements);
		this.drive = drive;
		this.localization = localization;
		this.telemetry = telemetry;
		this.eHistory = new ArrayList<>();

		if (telemetry != null) {
			telemetry.addData("[SYNCHROPATHER] Rotation desiredVelocity.getHeading()", 0);
			telemetry.addData("[SYNCHROPATHER] Rotation desiredState.getHeading()", 0);
			telemetry.addData("[SYNCHROPATHER] Rotation error.getHeading()", 0);
			telemetry.addData("[SYNCHROPATHER] robot.drive.turnVelocity", 0);
			telemetry.update();
		}
	}

	/**
	 * Creates a new RotationPlan object with the given Movements.
	 * @param drive the robot's DriveSubsystem.
	 * @param localization the robot's LocalizationSubsystem.
	 * @param movements
	 */
	public RotationPlan(DriveSubsystem drive, LocalizationSubsystem localization, Movement... movements) {
		this(drive, localization, null, movements);
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
		Pose2d currentPose = localization.getPose();
		RotationState currentState = new RotationState(currentPose.getHeading());

		// State error
		RotationState error = desiredState.minus(currentState);
		double e = normalizeAngle(error.getHeading());

		// Error derivatives
		double dedt = 0;
		eHistory.add(e);
		if (eHistory.size()>5) eHistory.remove(0);
		if (eHistory.size()==5) dedt = localization.stencil(eHistory);

		// Control output
		double u = 0;

		// Rotational PD
		u += kP*e + kD*dedt;

		// Feedforward
		u += kS*Math.signum(dv) + kV*dv + kA*da;

		// Set drive powers
		drive.turnVelocity = u;
		drive.driveFieldCentric(currentPose.getHeading()); // note: function overload
		drive.targetH = desiredState.getHeading();

		if (telemetry != null) {
			telemetry.addData("[SYNCHROPATHER] Rotation desiredVelocity.getHeading()", desiredVelocity.getHeading());
			telemetry.addData("[SYNCHROPATHER] Rotation desiredState.getHeading()", desiredState.getHeading());
			telemetry.addData("[SYNCHROPATHER] Rotation error.getHeading()", error.getHeading());
			telemetry.addData("[SYNCHROPATHER] robot.drive.turnVelocity", drive.turnVelocity);
		}
	}

	@Override
	public void stop() {
		drive.turnVelocity = 0;
		drive.stopController();
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
