package org.firstinspires.ftc.teamcode.synchropather.systems.translation;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotSystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LocalizationSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.DriveConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.MovementType;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.Movement;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.Plan;

import java.util.ArrayList;

/**
 * Object containing a sequence of Movements for translational drive.
 */
@Config
public class TranslationPlan extends Plan<TranslationState> {

	// Feedforward constants
	//TODO: TUNE
	public static double kS = 0;
	public static double kV = 1;
	public static double kA = 0.175;

	// Positional PD constants
	//TODO: TUNE
	public static double kP = 8;
	public static double kD = 0.125;

	// Error history array
	private final ArrayList<Double> exHistory;
	private final ArrayList<Double> eyHistory;

	private DriveSubsystem drive;
	private LocalizationSubsystem localization;
	private Telemetry telemetry;

	/**
	 * Creates a new TranslationPlan object with the given Movements.
	 * @param drive the robot's DriveSubsystem.
	 * @param localization the robot's LocalizationSubsystem.
	 * @param telemetry the robot's telemetry
	 * @param movements
	 */
	public TranslationPlan(DriveSubsystem drive, LocalizationSubsystem localization, Telemetry telemetry, Movement... movements) {
		super(MovementType.TRANSLATION, movements);
		this.drive = drive;
		this.localization = localization;
		this.telemetry = telemetry;
		this.exHistory = new ArrayList<>();
		this.eyHistory = new ArrayList<>();

		if (telemetry != null) {
			telemetry.addData("[SYNCHROPATHER] Translation desiredVelocity.getX()", 0);
			telemetry.addData("[SYNCHROPATHER] Translation desiredVelocity.getY()", 0);
			telemetry.addData("[SYNCHROPATHER] Translation ut.getX()", 0);
			telemetry.addData("[SYNCHROPATHER] Translation ut.getY()", 0);
			telemetry.addData("[SYNCHROPATHER] robot.drive.driveTheta", 0);
			telemetry.addData("[SYNCHROPATHER] robot.drive.driveSpeed", 0);
			telemetry.addData("[SYNCHROPATHER] currentPose.getHeading()", 0);
			telemetry.update();
		}
	}

	/**
	 * Creates a new TranslationPlan object with the given Movements.
	 * @param drive the robot's DriveSubsystem.
	 * @param localization the robot's LocalizationSubsystem.
	 * @param movements
	 */
	public TranslationPlan(DriveSubsystem drive, LocalizationSubsystem localization, Movement... movements) {
		this(drive, localization, null, movements);
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
		Pose2d currentPose = localization.getPose();
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
		if (exHistory.size()==5) dexdt = localization.stencil(exHistory);
		if (eyHistory.size()==5) deydt = localization.stencil(eyHistory);

		// Control output
		double ux = 0;
		double uy = 0;

		// Positional PD
		ux += kP*ex + kD*dexdt;
		uy += kP*ey + kD*deydt;

		// Feedforward
		ux += kS * Math.signum(dxv) + kV * dxv + kA * dxa;
		uy += kS * Math.signum(dyv) + kV * dyv + kA * dya;

		// Set drive parameters
		TranslationState ut = new TranslationState(ux, uy);
		TranslationState u_static = new TranslationState(kS, ut.theta(), true);
		ut = ut.plus(u_static);
		drive.driveTheta = ut.theta();
		drive.driveSpeed = ut.hypot();
		drive.driveFieldCentric(currentPose.getHeading());
		drive.targetX = desiredState.getX();
		drive.targetY = desiredState.getY();

		if (telemetry != null) {
			telemetry.addData("[SYNCHROPATHER] Translation desiredVelocity.getX()", desiredVelocity.getX());
			telemetry.addData("[SYNCHROPATHER] Translation desiredVelocity.getY()", desiredVelocity.getY());
			telemetry.addData("[SYNCHROPATHER] Translation ut.getX()", ut.getX());
			telemetry.addData("[SYNCHROPATHER] Translation ut.getY()", ut.getY());
			telemetry.addData("[SYNCHROPATHER] robot.drive.driveTheta", drive.driveTheta);
			telemetry.addData("[SYNCHROPATHER] robot.drive.driveSpeed", drive.driveSpeed);
			telemetry.addData("[SYNCHROPATHER] currentPose.getHeading()", currentPose.getHeading());
		}
	}

	@Override
	public void stop() {
		drive.driveTheta = 0;
		drive.driveSpeed = 0;
		drive.stopController();
	}

}
