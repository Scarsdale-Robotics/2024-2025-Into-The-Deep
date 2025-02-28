package org.firstinspires.ftc.teamcode.synchropather.systems.translation;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LocalizationSubsystem;
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

	// Positional SQUID constants
	//TODO: TUNE
	public static double kSQU = 6;
	public static double kI = 0;
	public static double kD = 0.125;

	public static double POWER_CACHE_THRESHOLD = 0.4;

	// Integrator variables
	private double intexdt = 0;
	private double inteydt = 0;

	// Error history array
	private final ArrayList<Double> exHistory;
	private final ArrayList<Double> eyHistory;
	private final ArrayList<Double> dtHistory;

	private DriveSubsystem drive;
	private LocalizationSubsystem localization;
	private ElapsedTime runtime;
	private Telemetry telemetry;
	private TranslationState lastDriveVelocity;

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
		this.dtHistory = new ArrayList<>();
		this.lastDriveVelocity = new TranslationState(0,0);

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
		exHistory.add(ex);
		eyHistory.add(ey);
		if (exHistory.size()>5) exHistory.remove(0);
		if (eyHistory.size()>5) eyHistory.remove(0);

		// Get delta time
		double deltaTime;
		boolean runtimeWasNull = false;
		if (runtime==null) {
			runtime = new ElapsedTime(0);
			deltaTime = 0;
			runtimeWasNull = true;
		} else {
			deltaTime = runtime.seconds();
			runtime.reset();
			dtHistory.add(deltaTime);
			if (dtHistory.size()>5) dtHistory.remove(0);
		}

		// Error integrals
		if (!runtimeWasNull) {
			intexdt += deltaTime * ex;
			inteydt += deltaTime * ey;
		}
		if (exHistory.size() > 1) {
			if (exHistory.get(exHistory.size() - 2) * ex <= 0) {
				// flush integral stack
				intexdt = 0;
			}
		}
		if (eyHistory.size() > 1) {
			if (eyHistory.get(eyHistory.size() - 2) * ey <= 0) {
				// flush integral stack
				inteydt = 0;
			}
		}
		if (kI != 0) {
			// Limit integrator to prevent windup
			double integralPowerThreshold = 0.25;
			double integralThresholdBound = Math.abs(integralPowerThreshold * TranslationConstants.MAX_VELOCITY / kI);
			intexdt = bound(intexdt, -integralThresholdBound, integralThresholdBound);
			inteydt = bound(inteydt, -integralThresholdBound, integralThresholdBound);
		}

		// Error derivatives
		double dexdt = 0;
		if (dtHistory.size()==5) {
			if (exHistory.size() == 5) {
				dexdt = stencil(exHistory);
			}
		}
		double deydt = 0;
		if (dtHistory.size()==5) {
			if (eyHistory.size() == 5) {
				deydt = stencil(eyHistory);
			}
		}


		// Positional SQUID
		double squx = Math.signum(ex)*Math.sqrt(Math.abs(ex));
		double squy = Math.signum(ey)*Math.sqrt(Math.abs(ey));
		double utx = kSQU*squx + kI*intexdt + kD*dexdt;
		double uty = kSQU*squy + kI*inteydt + kD*deydt;
		TranslationState ut = new TranslationState(utx, uty);

		// Feedforward
		double fux = kV * dxv + kA * dxa;
		double fuy = kV * dyv + kA * dya;
		TranslationState fu = new TranslationState(fux, fuy);
		TranslationState fu_static = new TranslationState(kS, fu.theta(), true)
				.times(Math.signum(fu.hypot())); // if fu is zero, then fu_static should be 0
		fu = fu.plus(fu_static);

		// Control output
		TranslationState u = ut.plus(fu);

		// Set drive parameters
		double speedDifference = u.minus(lastDriveVelocity).hypot();
		if (speedDifference >= POWER_CACHE_THRESHOLD || (approxEquiv(u.hypot(),0) && !approxEquiv(lastDriveVelocity.hypot(),0))) {
			drive.driveTheta = u.theta();
			drive.driveSpeed = u.hypot();
			drive.driveFieldCentric(currentPose.getHeading());
			lastDriveVelocity = u;
		}
		drive.targetX = desiredState.getX();
		drive.targetY = desiredState.getY();

		if (telemetry != null) {
			telemetry.addData("[SYNCHROPATHER] Translation desiredVelocity.getX()", desiredVelocity.getX());
			telemetry.addData("[SYNCHROPATHER] Translation desiredVelocity.getY()", desiredVelocity.getY());
			telemetry.addData("[SYNCHROPATHER] Translation u.getX()", u.getX());
			telemetry.addData("[SYNCHROPATHER] Translation u.getY()", u.getY());
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

	/**
	 * @param a The process value array.
	 * @return Approximated derivative according to the Five-Point stencil.
	 */
	public double stencil(ArrayList<Double> a) {
		double averageDeltaTime = dtHistory.stream().mapToDouble(aa -> aa).average().orElse(0);
		return (-a.get(4) + 8*a.get(3) - 8*a.get(1) + a.get(0)) /
				(12 * averageDeltaTime);
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

	private boolean approxEquiv(double a, double b) {
		return Math.abs(a-b) <= POWER_CACHE_THRESHOLD;
	}

}
