package org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;

import org.firstinspires.ftc.teamcode.synchropather.systems.MovementType;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.TimeSpan;

/**
 * Object containing a sequence of Movements for a single system.
 */
public abstract class Plan<T extends RobotState> {

	/**
	 * The sequence of Movements in this Plan.
	 */
	private ArrayList<Movement> movements;
	
	/**
	 * The elapsed time that indicates the target RobotState that calling loop() will correct to.
	 */
	private double targetTime;

	/**
	 * The type of Movements contained in this Plan.
	 */
	public final MovementType movementType;
	
	/**
	 * Creates a new Plan object with the given MovementType and Movements.
	 * @param movementType
	 * @param movements
	 */
	public Plan(MovementType movementType, Movement... movements) {
		this.movementType = movementType;
		this.targetTime = 0;
		this.movements = new ArrayList<>();
		
		// register Movements
		for (Movement movement : movements) {
			// throw error if wrong type
			if (movement.movementType != movementType) {
				throw new RuntimeException(String.format("Movement type %s does not match the type of this Plan %s", movement.movementType, movementType));
			}
			
			// append movement
			int index = checkValidity(movement);
			this.movements.add(index, movement);
		}
	}
	
	/**
	 * Check if the given Movement can be appended to the current collection and returns the index where it can be inserted.
	 * @param movement
	 * @return the index of insertion.
	 */
	private int checkValidity(Movement movement) {
		if (movements.size() == 0) return 0;
		
		// get index
		int index = Collections.binarySearch(movements, movement, Comparator.comparingDouble(Movement::getStartTime));
		if (index < 0) index = -(index + 1);
		
		// throw error if there are overlaps
		double s = movement.getStartTime(), e = movement.getEndTime();
		if (index > 0 && s < movements.get(index-1).getEndTime()) {
			throw new RuntimeException(String.format("Movement %s overlaps with an earlier movement", movement.getDisplayName()));
		}
		if (index < movements.size() && movements.get(index).getStartTime() < e) {
			throw new RuntimeException(String.format("Movement %s overlaps with a later movement", movement.getDisplayName()));
		}
		
		return index;
	}
	
	/**
	 * Calling loop() will control this Plan's robot subsystem to the RobotState at targetTime.
	 */
	public abstract void loop();
	
	/**
	 * Sets targetTime of this Plan to the given elapsedTime.
	 * @param elapsedTime
	 */
	public void setTarget(double elapsedTime) {
		this.targetTime = elapsedTime;
	}

	/**
	 * @return the RobotState at the current targetTime.
	 */
	public T getCurrentState() {
		return getState(targetTime);
	}

	/**
	 * Returns the RobotState at the given elapsedTime.
	 * @param elapsedTime
	 * @return the indicated RobotState.
	 */
	@SuppressWarnings("unchecked")
	public T getState(double elapsedTime) {
		if (movements.size() == 0) throw new RuntimeException("Tried to call getState() with an empty Movements list");

		// get index of the movement at targetTime
		int index = 0;
		while (index+1 < movements.size() && elapsedTime >= movements.get(index+1).getStartTime()) {
			index++;
		}

		return (T) movements.get(index).getState(elapsedTime);
	}

	/**
	 * Returns the velocity RobotState at the given elapsedTime.
	 * @param elapsedTime
	 * @return the indicated velocity RobotState.
	 */
	@SuppressWarnings("unchecked")
	public T getVelocity(double elapsedTime) {
		if (movements.size() == 0) throw new RuntimeException("Tried to call getState() with an empty Movements list");

		// get index of the movement at targetTime
		int index = 0;
		while (index+1 < movements.size() && elapsedTime >= movements.get(index+1).getStartTime()) {
			index++;
		}

		return (T) movements.get(index).getVelocity(elapsedTime);
	}

	/**
	 * @return the minimum duration needed to execute all Movements within this Plan
	 */
	public double getDuration() {
		if (movements.size() == 0) return 0;
		return movements.get(movements.size()-1).getEndTime();
	}

}
