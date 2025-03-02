package org.firstinspires.ftc.teamcode.synchropather.systems.rotation;

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
 * Object containing a sequence of Movements for rotational drive.
 */
@Config
public class RotationPlan extends Plan<RotationState> {

	// Feedforward constants
	//TODO: TUNE
	public static double kS = 0;
	public static double kV = 1;
	public static double kA = 0.1;

	// SQUID constants
	//TODO: TUNE
	public static double kSQU = 1;
	public static double kI = 0;
	public static double kD = 0;

	public static double POWER_CACHE_THRESHOLD = 0.04;

	// Integrator variable
	private double intedt = 0;

	// Error history array
	private final ArrayList<Double> eHistory;
	private final ArrayList<Double> dtHistory;

	private DriveSubsystem drive;
	private LocalizationSubsystem localization;
	private ElapsedTime runtime;
	private Telemetry telemetry;
	private double lastAngularVelocity;

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
		this.dtHistory = new ArrayList<>();
		this.lastAngularVelocity = Double.MIN_VALUE;

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
		eHistory.add(e);
		if (eHistory.size()>5) eHistory.remove(0);

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
			intedt += deltaTime * e;
		}
		if (eHistory.size() > 1) {
			if (eHistory.get(eHistory.size() - 2) * e <= 0) {
				// flush integral stack
				intedt = 0;
			}
		}
		if (kI != 0) {
			// Limit integrator to prevent windup
			double integralPowerThreshold = 0.25;
			double integralThresholdBound = Math.abs(integralPowerThreshold * RotationConstants.MAX_ANGULAR_VELOCITY / kI);
			intedt = bound(intedt, -integralThresholdBound, integralThresholdBound);
		}

		// Error derivatives
		double dedt = 0;
		if (dtHistory.size()==5) {
			if (eHistory.size() == 5) {
				dedt = stencil(eHistory);
			}
		}

		// Control output
		double u = 0;

		// Rotational SQUID
		double squ = Math.signum(e)*Math.sqrt(Math.abs(e));
		u += kSQU*squ + kI*intedt + kD*dedt;

		// Feedforward
		u += kS*Math.signum(dv) + kV*dv + kA*da;

		// Set drive powers
		if (Math.abs(u - lastAngularVelocity) >= POWER_CACHE_THRESHOLD || (approxEquiv(u,0) && !approxEquiv(lastAngularVelocity,0))) {
			drive.turnVelocity = u;
			drive.driveFieldCentric(currentPose.getHeading());
			lastAngularVelocity = u;
		}
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
