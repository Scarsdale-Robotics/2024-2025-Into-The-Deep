package org.firstinspires.ftc.teamcode.synchropather.systems.translation;

import org.firstinspires.ftc.teamcode.synchropather.systems.MovementType;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.Movement;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.Plan;

/**
 * Object containing a sequence of Movements for translational drive.
 */
public class TranslationPlan extends Plan<TranslationState> {

	/**
	 * Creates a new TranslationPlan object with the given Movements.
	 * @param movements
	 */
	public TranslationPlan(Movement... movements) {
		super(MovementType.TRANSLATION, movements);
	}

	/**
	 * Controls the translation output of the robot to the TranslationState at the elapsedTime.
	 */
	@Override
	public void loop() {
		// TODO Auto-generated method stub
		TranslationState state = getCurrentState();
		
	}

}
