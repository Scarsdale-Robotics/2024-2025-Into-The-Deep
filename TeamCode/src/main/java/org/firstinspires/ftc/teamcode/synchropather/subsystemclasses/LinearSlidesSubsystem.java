package org.firstinspires.ftc.teamcode.synchropather.subsystemclasses;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.synchropather.systems.extendo.ExtendoConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.lift.LiftConstants;

@Config
public class LinearSlidesSubsystem {

    private final Motor extendo;
    private final Motor leftLift;
    private final Motor rightLift;

    public final Telemetry telemetry;

    public static double extendoOffset = 0; // TODO: REMOVE!!!!!!!!!!! (AFTER WIRING IS FIXED)

    private double extendoPosition;
    private double leftLiftPosition;
    private double rightLiftPosition;

    public LinearSlidesSubsystem(Motor extendo, Motor leftLift, Motor rightLift, Telemetry telemetry) {
        this.extendo = extendo;
        this.leftLift = leftLift;
        this.rightLift = rightLift;
        this.telemetry = telemetry;
        update();
    }

    /**
     * Performs a bulk hardware call and gets current motor positions.
     */
    public void update() {
        extendoPosition = extendo.getCurrentPosition();
        leftLiftPosition = leftLift.getCurrentPosition();
        rightLiftPosition = rightLift.getCurrentPosition();
    }

    /**
     * @param power between [-1,1]
     */
    public void setExtendoPower(double power) {
        extendo.motor.setPower(power);
    }

    /**
     * @param power between [-1,1]
     */
    public void setLiftPowers(double power) {
        setLeftLiftPower(power);
        setRightLiftPower(power);
    }

    /**
     * @param power between [-1,1]
     */
    public void setLeftLiftPower(double power) {
        leftLift.motor.setPower(power);
    }

    /**
     * @param power between [-1,1]
     */
    public void setRightLiftPower(double power) {
        rightLift.motor.setPower(power);
    }

    /**
     * Sets extendo power to zero.
     */
    public void stopExtendo() {
        extendo.motor.setPower(0);
    }

    /**
     * Sets both lift powers to zero.
     */
    public void stopLifts() {
        stopLeftLift();
        stopRightLift();
    }

    /**
     * Sets left lift power to zero.
     */
    public void stopLeftLift() {
        leftLift.motor.setPower(0);
    }

    /**
     * Sets right lift power to zero.
     */
    public void stopRightLift() {
        rightLift.motor.setPower(0);
    }

    /**
     * @return extendo length in inches, where fully retracted = 0
     */
    public double getExtendoPosition() {
        return extendoOffset + extendoPosition / ExtendoConstants.TICKS_PER_INCH;
    }

    /**
     * @return the current distance from the claw (at pickup position) to the robot's center of rotation.
     */
    public double getExtendoClawPosition() {
        return ExtendoConstants.CLAW_OFFSET_DISTANCE + getExtendoPosition();
    }

    /**
     * @return left lift height in inches
     */
    public double getLeftLiftPosition() {
        return leftLiftPosition / LiftConstants.TICKS_PER_INCH;
    }

    /**
     * @return right lift height in inches
     */
    public double getRightLiftPosition() {
        return rightLiftPosition / LiftConstants.TICKS_PER_INCH;
    }

}
