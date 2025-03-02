package org.firstinspires.ftc.teamcode.synchropather.systems.translation.movements;

import org.firstinspires.ftc.teamcode.synchropather.systems.MovementType;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.TimeSpan;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.motion_profiles.SymmetricMotionProfile1D;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.motion_profiles.TrapezoidalMotionProfile1D;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.Movement;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.TranslationConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.TranslationState;

/**
 * Movement for planning a linear translation.
 */
public class DifferentAccelerationsTranslation extends Movement {

	private double distance, minDuration;
	private TranslationState start, end;
	private TrapezoidalMotionProfile1D motionProfile;


	/**
	 * Creates a new LinearTranslation object with a given start and end TranslationState allotted for the given TimeSpan.
	 * @param timeSpan
	 * @param start
	 * @param end
	 */
	public DifferentAccelerationsTranslation(TimeSpan timeSpan, TranslationState start, TranslationState end, double acceleration, double deceleration) {
		super(timeSpan, MovementType.TRANSLATION);
		this.start = start;
		this.end = end;
		init(false, -1, acceleration, deceleration);
	}

	/**
	 * Creates a new LinearTranslation object with a given start and end TranslationState at the given startTime.
	 * @param startTime
	 * @param start
	 * @param end
	 */
	public DifferentAccelerationsTranslation(double startTime, TranslationState start, TranslationState end, double acceleration, double deceleration) {
		super(MovementType.TRANSLATION);
		this.start = start;
		this.end = end;
		init(true, startTime, acceleration, deceleration);
	}

	/**
	 * @return the indicated TranslationState.
	 */
	@Override
	public TranslationState getState(double elapsedTime) {
		double t = distance!=0 ? motionProfile.getDisplacement(elapsedTime) / distance : 0;

		double q0 = 1 - t;
		double q1 = t;

		// linear interpolation
		return start.times(q0).plus(end.times(q1));
	}

	/**
	 * @return the indicated velocity TranslationState.
	 */
	@Override
	public TranslationState getVelocity(double elapsedTime) {
		double theta = end.minus(start).theta();
		double speed = motionProfile.getVelocity(elapsedTime);

		// scaled velocity vector
		return new TranslationState(speed, theta, true);
	}

	/**
	 * @return the indicated acceleration TranslationState.
	 */
	@Override
	public TranslationState getAcceleration(double elapsedTime) {
		double theta = end.minus(start).theta();
		double speed = motionProfile.getAcceleration(elapsedTime);

		// scaled acceleration vector
		return new TranslationState(speed, theta, true);
	}
	
	@Override
	public double getMinDuration() {
		return minDuration;
	}

	/**
	 * @return the TranslationState of this Movement at the start time.
	 */
	@Override
	public TranslationState getStartState() {
		return start;
	}

	/**
	 * @return the TranslationState of this Movement at the end time.
	 */
	@Override
	public TranslationState getEndState() {
		return end;
	}

	/**
	 * @return "LinearTranslation"
	 */
	@Override
	public String getDisplayName() {
		return "LinearTranslation";
	}
	
	/**
	 * Calculates total time.
	 */
	private void init(boolean startTimeConstructor, double startTime, double acceleration, double deceleration) {
		distance = end.minus(start).hypot();

		double v_max = TranslationConstants.MAX_VELOCITY;
		double a_max_1 = acceleration;
		double a_max_2 = deceleration;

		motionProfile = new TrapezoidalMotionProfile1D(distance, startTime, v_max, a_max_1, a_max_2);
		if (startTimeConstructor) {
			timeSpan = motionProfile.getTimeSpan();
		}
		minDuration = motionProfile.getMinDuration();
	}

}
