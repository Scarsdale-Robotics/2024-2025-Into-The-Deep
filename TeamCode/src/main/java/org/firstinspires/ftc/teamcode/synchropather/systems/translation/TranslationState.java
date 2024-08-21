package org.firstinspires.ftc.teamcode.synchropather.systems.translation;

import org.firstinspires.ftc.teamcode.synchropather.systems.MovementType;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.RobotState;

/**
 * Object containing a Cartesian coordinate pair representing the global translation of the robot on the field.
 */
public class TranslationState extends RobotState {

	public static final TranslationState zero = new TranslationState(0,0);
	private final double x;
	private final double y;
	
	/**
	 * Creates a new TranslationState object with the given polar coordinates.
	 * @param r inches
	 * @param theta radians
	 * @param polar set to true for polar coordinates, false for Cartesian coordinates
	 */
	public TranslationState(double r, double theta, boolean polar) {
		if (polar) {
			this.x = r * Math.cos(theta);
			this.y = r * Math.sin(theta);
	    } else {
	    	this.x = r;
	    	this.y = theta;
	    }
	}
	
	/**
	 * Creates a new TranslationState object with the given Cartesian coordinates.
	 * @param x inches
	 * @param y inches
	 */
	public TranslationState(double x, double y) {
		this.x = x;
	    this.y = y;
	}
	
	/**
	 * @return the x coordinate in inches.
	 */
	public double getX() {
		return x;
	}

	/**
	 * @return the y coordinate in inches.
	 */
	public double getY() {
		return y;
	}

	/**
	 * Returns whether or not the coordinates of this TranslationState equals those of the given.
	 * @param other
	 * @return true if the coordinates are equal.
	 */
	public boolean equals(TranslationState other) {
		return Math.abs(getX() - other.getX()) < 1e-6 &&
				Math.abs(getY() - other.getY()) < 1e-6;
	}

	/**
	 * @return the hypotenuse length sqrt(<i>x</i><sup>2</sup>&nbsp;+<i>y</i><sup>2</sup>).
	 */
	public double hypot() {
		return Math.hypot(x, y);
	}

	/**
	 * @return the angle that the line segment connecting this TranslationPose to the origin makes with the positive x axis.
	 */
	public double theta() {
		return Math.atan2(y, x);
	}
	
	/**
	 * Adds two TranslationStates.
	 * @param addend
	 * @return the sum as a TranslationState.
	 */
	public TranslationState plus(TranslationState addend) {
		return new TranslationState(x + addend.getX(), y + addend.getY());
	}
	
	/**
	 * Subtracts two TranslationStates.
	 * @param subtrahend
	 * @return the difference as a TranslationState.
	 */
	public TranslationState minus(TranslationState subtrahend) {
		return new TranslationState(x - subtrahend.getX(), y - subtrahend.getY());
	}
	
	/**
	 * Multiplies by a constant factor.
	 * @param factor
	 * @return the product as a TranslationState.
	 */
	public TranslationState times(double factor) {
		return new TranslationState(x * factor, y * factor);
	}

	@Override
	/**
	 * @return a String containing the ordered pair (x,y) in inches.
	 */
	public String toString() {
		String res = String.format("(%sin,%sin)", x, y);
		return res;
	}

	@Override
	/**
	 * @return "Translation"
	 */
	public String getDisplayName() {
		return "Translation";
	}

}
