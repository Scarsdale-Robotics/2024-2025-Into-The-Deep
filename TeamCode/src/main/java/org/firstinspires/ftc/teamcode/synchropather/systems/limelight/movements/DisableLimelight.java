package org.firstinspires.ftc.teamcode.synchropather.systems.limelight.movements;

import org.firstinspires.ftc.teamcode.synchropather.systems.MovementType;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.TimeSpan;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.Movement;
import org.firstinspires.ftc.teamcode.synchropather.systems.limelight.LimelightPipeline;
import org.firstinspires.ftc.teamcode.synchropather.systems.limelight.LimelightState;

public class DisableLimelight extends Movement {

    /**
     * Creates a new DisableLimelight Movement with the given start time.
     * @param startTime seconds.
     */
    public DisableLimelight(TimeSpan timeSpan) {
        super(timeSpan, MovementType.LIMELIGHT);
    }

    @Override
    public double getMinDuration() {
        return 0;
    }

    @Override
    public LimelightState getState(double elapsedTime) {
        return new LimelightState();
    }

    @Override
    public LimelightState getVelocity(double elapsedTime) {
        return null;
    }

    @Override
    public LimelightState getAcceleration(double elapsedTime) {
        return null;
    }

    @Override
    public LimelightState getStartState() {
        return new LimelightState();
    }

    @Override
    public LimelightState getEndState() {
        return new LimelightState();
    }

    @Override
    public String getDisplayName() {
        return "DisableLimelight";
    }
}
