package org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses;

import org.firstinspires.ftc.teamcode.synchropather.systems.MovementType;

/**
 * Object containing variables representing the state of an independent subsystem on the Robot.
 */
public abstract class RobotState {

	/**
	 * @return a String containing the variables of this RobotState.
	 */
	public abstract String toString();
	
	/**
	 * @return a String representing this RobotState class.
	 */
	public abstract String getDisplayName();
	
}
