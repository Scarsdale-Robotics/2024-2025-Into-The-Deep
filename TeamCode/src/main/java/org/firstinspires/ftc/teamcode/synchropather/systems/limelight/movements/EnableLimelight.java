package org.firstinspires.ftc.teamcode.synchropather.systems.limelight.movements;

import org.firstinspires.ftc.teamcode.synchropather.systems.MovementType;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.TimeSpan;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.Movement;
import org.firstinspires.ftc.teamcode.synchropather.systems.limelight.LimelightPipeline;
import org.firstinspires.ftc.teamcode.synchropather.systems.limelight.LimelightState;

public class EnableLimelight extends Movement {

    private LimelightPipeline limelightPipeline;

    /**
     * Creates a new EnableLimelight Movement with the given start time and LimelightPipeline.
     * @param startTime seconds.
     * @param limelightPipeline
     */
    public EnableLimelight(TimeSpan timeSpan, LimelightPipeline limelightPipeline) {
        super(timeSpan, MovementType.LIMELIGHT);
        this.limelightPipeline = limelightPipeline;
    }

    @Override
    public double getMinDuration() {
        return 0;
    }

    @Override
    public LimelightState getState(double elapsedTime) {
        return new LimelightState(limelightPipeline);
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
        return new LimelightState(limelightPipeline);
    }

    @Override
    public LimelightState getEndState() {
        return new LimelightState(limelightPipeline);
    }

    @Override
    public String getDisplayName() {
        return "EnableLimelight";
    }
}
