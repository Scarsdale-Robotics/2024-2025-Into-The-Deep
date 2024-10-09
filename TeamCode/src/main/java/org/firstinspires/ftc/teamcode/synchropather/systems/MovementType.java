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
		 * A Movement involving the global lift height.
		 */
		LIFT(),
		/**
		 * A Movement involving the elbow.
		 */
		ELBOW(),
		/**
		 * A Movement involving the elbow.
		 */
		CLAW();

		MovementType() {};
}