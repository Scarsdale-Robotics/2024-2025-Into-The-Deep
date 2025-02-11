package org.firstinspires.ftc.teamcode.synchropather.systems;

/**
 * The type of Movement.
 */
public enum MovementType {
		
	/**
	 * A Movement involving the global x and y coordinates.
	 */
	TRANSLATION(),

	/**
	 * A Movement involving the global heading.
	 */
	ROTATION(),
	/**
	 * A Movement involving the vertical lift.
	 */
	LIFT(),
	/**
	 * A Movement involving the horizontal extension slide.
	 */
	EXTENDO(),
	/**
	 * A Movement involving the horizontal arm.
	 */
	HORIZONTAL_ARM(),
	/**
	 * A Movement involving the horizontal wrist.
	 */
	HORIZONTAL_WRIST(),
	/**
	 * A Movement involving the horizontal claw.
	 */
	HORIZONTAL_CLAW(),
	/**
	 * A Movement involving the clipbot's magazine
	 */
	CLIPBOT_MAGAZINE();

	MovementType() {};
}