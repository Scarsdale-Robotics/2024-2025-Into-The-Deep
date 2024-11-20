package org.firstinspires.ftc.teamcode.synchropather.systems.__util__;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.synchropather.systems.MovementType;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.Plan;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.RobotState;

/**
 * An object that contains other Plans for synchronizing robot subsystems.
 */
public class Synchronizer {

	private Plan[] plans;

	/**
	 * The elapsed time that determines the state of every Plan.
	 */
	private ElapsedTime runtime;

	/**
	 * The time when the synchronizer started, in seconds.
	 */
	private double startTime;

	/**
	 * Whether or not the synchronizer is running.
	 */
	private boolean running;

	/**
	 * Creates a new Synchronizer object with the given Plans.
	 * @param plans
	 */
	public Synchronizer(Plan... plans) {
		this.plans = plans;
		this.runtime = new ElapsedTime(0);
		this.startTime = 0;
		this.running = false;
	}

	/**
	 * Resets the elapsed time to zero and starts the timer.
	 */
	public void start() {
		startTime = runtime.seconds();
		running = true;
	}

	/**
	 * @return whether or not this synchronizer is running.
	 */
	public boolean getIsRunning() {
		return running;
	}

	/**
	 * Resets the elapsed time to the given elapsed time and immediately starts the timer.
	 */
	public void start(double elapsedTime) {
		startTime = runtime.seconds() - elapsedTime;
		running = true;
	}

	/**
	 * Sets the target of all plans contained within this Synchronizer to the given elapsedTime and calls loop().
	 * @return whether or not the synchronizer should still be running.
	 */
	public boolean update() {
		if (!running) throw new RuntimeException("Synchronizer: tried calling update() before calling start()!");
		double elapsedTime = runtime.seconds() - startTime;
		for (Plan plan : plans) {
			plan.setTarget(elapsedTime);
			plan.loop();
		}
		return elapsedTime < getDuration();
	}

	/**
	 * @return the current elapsed time if this Synchronizer is running.
	 */
	public double getElapsedTime() {
		if (!running) throw new RuntimeException("Synchronizer: tried calling getElapsedTime() before calling start()!");
		return runtime.seconds() - startTime;
	}

	/**
	 * Stops every subsystem and sets running to false.
	 */
	public void stop() {
		for (Plan plan : plans) {
			plan.stop();
		}
		running = false;
	}

	/**
	 * Gets the RobotState at the given elapsedTime within the Plan of the given movementType.
	 * @param movementType
	 * @param elapsedTime
	 * @return the indicated RobotState, or null if the Plan does not exist.
	 */
	@SuppressWarnings("unchecked")
	public RobotState getState(MovementType movementType, double elapsedTime) {
		for (Plan plan : plans) {
			if (plan.movementType == movementType) {
				return plan.getState(elapsedTime);
			}
		}
		return null;
	}

	/**
	 * Gets the RobotState at the current elapsed time within the Plan of the given movementType.
	 * @param movementType
	 * @return the indicated RobotState, or null if the Plan does not exist.
	 */
	public RobotState getState(MovementType movementType) {
		double elapsedTime = runtime.seconds() - startTime;
		return getState(movementType, elapsedTime);
	}

	/**
	 * Gets the velocity RobotState at the given elapsedTime within the Plan of the given movementType.
	 * @param movementType
	 * @param elapsedTime
	 * @return the indicated velocity RobotState, or null if the Plan does not exist.
	 */
	@SuppressWarnings("unchecked")
	public RobotState getVelocity(MovementType movementType, double elapsedTime) {
		for (Plan plan : plans) {
			if (plan.movementType == movementType) {
				return plan.getVelocity(elapsedTime);
			}
		}
		return null;
	}

	/**
	 * Gets the velocity RobotState at the current elapsed time within the Plan of the given movementType.
	 * @param movementType
	 * @return the indicated velocity RobotState, or null if the Plan does not exist.
	 */
	public RobotState getVelocity(MovementType movementType) {
		double elapsedTime = runtime.seconds() - startTime;
		return getVelocity(movementType, elapsedTime);
	}

	/**
	 * Gets the acceleration RobotState at the given elapsedTime within the Plan of the given movementType.
	 * @param movementType
	 * @param elapsedTime
	 * @return the indicated acceleration RobotState, or null if the Plan does not exist.
	 */
	@SuppressWarnings("unchecked")
	public RobotState getAcceleration(MovementType movementType, double elapsedTime) {
		for (Plan plan : plans) {
			if (plan.movementType == movementType) {
				return plan.getAcceleration(elapsedTime);
			}
		}
		return null;
	}

	/**
	 * Gets the acceleration RobotState at the current elapsed time within the Plan of the given movementType.
	 * @param movementType
	 * @return the indicated acceleration RobotState, or null if the Plan does not exist.
	 */
	public RobotState getAcceleration(MovementType movementType) {
		double elapsedTime = runtime.seconds() - startTime;
		return getAcceleration(movementType, elapsedTime);
	}

	/**
	 * @return the minimum duration needed to execute all Plans contained within this Synchronizer.
	 */
	public double getDuration() {
		double max = -1;
		for (Plan plan : plans) {
			max = Math.max(max, plan.getDuration());
		}
		return max;
	}

}
