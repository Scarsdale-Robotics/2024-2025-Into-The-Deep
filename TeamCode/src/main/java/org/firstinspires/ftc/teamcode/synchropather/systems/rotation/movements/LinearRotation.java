package org.firstinspires.ftc.teamcode.synchropather.systems.rotation.movements;

import org.firstinspires.ftc.teamcode.synchropather.systems.MovementType;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.TimeSpan;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.motion_profiles.SymmetricMotionProfile1D;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.Movement;
import org.firstinspires.ftc.teamcode.synchropather.systems.rotation.RotationConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.rotation.RotationState;

/**
 * Movement for planning a linear rotation.
 */
public class LinearRotation extends Movement {
	
	private double distance, minDuration;
	private RotationState start, end;
	private SymmetricMotionProfile1D motionProfile;

	/**
	 * Creates a new LinearRotation object with a given start and end RotationState allotted for the given TimeSpan.
	 * @param timeSpan
	 * @param start
	 * @param end
	 */
	public LinearRotation(TimeSpan timeSpan, RotationState start, RotationState end) {
		super(timeSpan, MovementType.ROTATION);
		this.start = start;
		this.end = end;
		init(false, -1);
	}

	/**
	 * Creates a new LinearRotation object with a given start and end RotationState at the given startTime.
	 * @param startTime
	 * @param start
	 * @param end
	 */
	public LinearRotation(double startTime, RotationState start, RotationState end) {
		super(MovementType.ROTATION);
		this.start = start;
		this.end = end;
		init(true, startTime);
	}


	@Override
	public double getMinDuration() {
		return minDuration;
	}

	/**
	 * @return the indicated RotationState.
	 */
	@Override
	public RotationState getState(double elapsedTime) {
		double t = distance!=0 ? motionProfile.getDisplacement(elapsedTime) / distance : 0;

		double q0 = 1 - t;
		double q1 = t;

		// linear interpolation
		return start.times(q0).plus(end.times(q1));
	}

	/**
	 * @return the indicated velocity RotationState.
	 */
	@Override
	public RotationState getVelocity(double elapsedTime) {
		double sign = end.minus(start).sign();
		double speed = motionProfile.getVelocity(elapsedTime);

		// scaled velocity vector
		return new RotationState(sign * speed);
	}

	/**
	 * @return the indicated acceleration RotationState.
	 */
	@Override
	public RotationState getAcceleration(double elapsedTime) {
		double sign = end.minus(start).sign();
		double speed = motionProfile.getAcceleration(elapsedTime);

		// scaled acceleration vector
		return new RotationState(sign * speed);
	}

	/**
	 * @return the RotationState of this Movement at the start time.
	 */
	@Override
	public RotationState getStartState() {
		return start;
	}

	/**
	 * @return the RotationState of this Movement at the end time.
	 */
	@Override
	public RotationState getEndState() {
		return end;
	}

	/**
	 * @return "LinearRotation"
	 */
	@Override
	public String getDisplayName() {
		return "LinearRotation";
	}
	
	/**
	 * Calculates total time.
	 */
	private void init(boolean startTimeConstructor, double startTime) {
		distance = end.minus(start).abs();

		double v_max = RotationConstants.MAX_ANGULAR_VELOCITY;
		double a_max = RotationConstants.MAX_ANGULAR_ACCELERATION;

		if (startTimeConstructor) {
			motionProfile = new SymmetricMotionProfile1D(distance, startTime, v_max, a_max);
			timeSpan = motionProfile.getTimeSpan();
		} else {
			motionProfile = new SymmetricMotionProfile1D(distance, timeSpan, v_max, a_max);
		}
		
		minDuration = motionProfile.getMinDuration();
	}

}
