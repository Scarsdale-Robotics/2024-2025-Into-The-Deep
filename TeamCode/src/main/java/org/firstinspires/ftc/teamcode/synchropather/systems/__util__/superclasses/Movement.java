package org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses;

import org.firstinspires.ftc.teamcode.synchropather.systems.MovementType;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.TimeSpan;

/**
 * Object containing the motion plan of a single Movement with respect to elapsed time.
 */
public abstract class Movement {

	/**
	 * The duration of time this Movement is allotted for.
	 */
	protected TimeSpan timeSpan;
	
	/**
	 * The type of this movement.
	 */
	public final MovementType movementType;
	
	/**
	 * Creates a new Movement with the given MovementType and TimeSpan.
	 * @param movementType
	 * @param timeSpan
	 */
	protected Movement(TimeSpan timeSpan, MovementType movementType) {
		this.movementType = movementType;
		this.timeSpan = timeSpan;
	}

	/**
	 * Creates a new Movement with the given MovementType.
	 * @param movementType
	 */
	protected Movement(MovementType movementType) {
		this.movementType = movementType;
	}


	/**
	 * @return the timestamp for when this Movement starts.
	 */
	public double getStartTime() {
		return timeSpan.getStartTime();
	}

	/**
	 * @return the timestamp for when this Movement ends.
	 */
	public double getEndTime() {
		return timeSpan.getEndTime();
	}

	/**
	 * @return the minimum runtime needed for this Movement.
	 */
	public abstract double getMinDuration();

	/**
	 * @return the total allotted runtime for this Movement.
	 */
	public double getDuration() {
		return timeSpan.getDuration();
	}

	/**
	 * Sets the allotted timeSpan of this Movement to the given timeSpan.
	 */
	public void setTimeSpan(TimeSpan timeSpan) {
		this.timeSpan = timeSpan;
	}

	/**
	 * Returns the robot's desired RobotState in this Movement at the given elapsed time.
	 * @param elapsedTime
	 * @return the indicated RobotState.
	 */
	public abstract RobotState getState(double elapsedTime);

	/**
	 * Returns the robot's desired velocity RobotState in this Movement at the given elapsed time.
	 * @param elapsedTime
	 * @return the indicated velocity RobotState.
	 */
	public abstract RobotState getVelocity(double elapsedTime);

	/**
	 * Returns the robot's desired acceleration RobotState in this Movement at the given elapsed time.
	 * @param elapsedTime
	 * @return the indicated acceleration RobotState.
	 */
	public abstract RobotState getAcceleration(double elapsedTime);

	/**
	 * @return the RobotState of this Movement at time zero.
	 */
	public abstract RobotState getStartState();

	/**
	 * @return the RobotState reached by the end of this Movement.
	 */
	public abstract RobotState getEndState();

	/**
	 * @return a String representing the class name of this Movement.
	 */
	public abstract String getDisplayName();

	/**
	 * @return this Movement's TimeSpan.
	 */
	public TimeSpan getTimeSpan() {
		return timeSpan;
	}

}
