package org.firstinspires.ftc.teamcode.synchropather.systems.rotation;

import org.firstinspires.ftc.teamcode.synchropather.systems.MovementType;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.Movement;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.Plan;

/**
 * Object containing a sequence of Movements for rotational drive.
 */
public class RotationPlan extends Plan<RotationState> {

	/**
	 * Creates a new RotationPlan object with the given Movements.
	 * @param movements
	 */
	public RotationPlan(Movement... movements) {
		super(MovementType.ROTATION, movements);
	}

	/**
	 * Controls the rotation output of the robot to the RotationState at the elapsedTime.
	 */
	@Override
	public void loop() {
		// TODO Auto-generated method stub
		
	}

}
