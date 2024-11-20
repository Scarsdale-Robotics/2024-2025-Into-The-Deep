package org.firstinspires.ftc.teamcode.synchropather.systems.translation;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.teamcode.RobotSystem;
import org.firstinspires.ftc.teamcode.opmodes.calibration.Drawing;
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
	public static double kD = 1;

	// Error history array
	private final ArrayList<Double> exHistory;
	private final ArrayList<Double> eyHistory;

	private RobotSystem robot;

	/**
	 * Creates a new TranslationPlan object with the given Movements.
	 * @param robot The RobotSystem containing a DriveSubsystem.
	 * @param movements
	 */
	public TranslationPlan(RobotSystem robot, Movement... movements) {
		super(MovementType.TRANSLATION, movements);
		this.robot = robot;
		this.exHistory = new ArrayList<>();
		this.eyHistory = new ArrayList<>();

		robot.telemetry.addData("[SYNCHROPATHER] Translation desiredVelocity.getX()", 0);
		robot.telemetry.addData("[SYNCHROPATHER] Translation desiredVelocity.getY()", 0);
		robot.telemetry.addData("[SYNCHROPATHER] Translation ut.getX()", 0);
		robot.telemetry.addData("[SYNCHROPATHER] Translation ut.getY()", 0);
		robot.telemetry.addData("[SYNCHROPATHER] robot.drive.driveTheta", 0);
		robot.telemetry.addData("[SYNCHROPATHER] robot.drive.driveSpeed", 0);
		robot.telemetry.addData("[SYNCHROPATHER] currentPose.getHeading()", 0);
		robot.telemetry.update();
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
		ux += kS * Math.signum(dxv) + kV * dxv + kA * dxa;
		uy += kS * Math.signum(dyv) + kV * dyv + kA * dya;

		// Set drive parameters
		TranslationState ut = new TranslationState(ux, uy);
		TranslationState u_static = new TranslationState(kS, ut.theta(), true);
		ut = ut.plus(u_static);
		robot.drive.driveTheta = ut.theta();
		robot.drive.driveSpeed = ut.hypot();
		robot.drive.driveFieldCentric(currentPose.getHeading());
		robot.drive.targetX = desiredState.getX();
		robot.drive.targetY = desiredState.getY();

		robot.telemetry.addData("[SYNCHROPATHER] Translation desiredVelocity.getX()", desiredVelocity.getX());
		robot.telemetry.addData("[SYNCHROPATHER] Translation desiredVelocity.getY()", desiredVelocity.getY());
		robot.telemetry.addData("[SYNCHROPATHER] Translation ut.getX()", ut.getX());
		robot.telemetry.addData("[SYNCHROPATHER] Translation ut.getY()", ut.getY());
		robot.telemetry.addData("[SYNCHROPATHER] robot.drive.driveTheta", robot.drive.driveTheta);
		robot.telemetry.addData("[SYNCHROPATHER] robot.drive.driveSpeed", robot.drive.driveSpeed);
		robot.telemetry.addData("[SYNCHROPATHER] currentPose.getHeading()", currentPose.getHeading());

	}

	@Override
	public void stop() {
		robot.drive.driveTheta = 0;
		robot.drive.driveSpeed = 0;
		robot.drive.stopController();
	}

}
