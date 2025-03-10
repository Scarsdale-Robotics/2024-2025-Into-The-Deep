package org.firstinspires.ftc.teamcode.synchropather.systems.limelight;

import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.RobotState;

public class LimelightState extends RobotState {

    private final boolean enabled;
    private final LimelightPipeline limelightPipeline;

    /**
     * Creates a new LimelightState with the given parameters.
     * @param enabled if the limelight should be running a pipeline.
     * @param limelightPipeline the pipeline that the limelight should be running.
     */
    public LimelightState(boolean enabled, LimelightPipeline limelightPipeline) {
        this.enabled = enabled;
        this.limelightPipeline = limelightPipeline;
    }

    /**
     * Creates a new DISABLED LimelightState.
     */
    public LimelightState() {
        this(false, LimelightPipeline.NONE);
    }

    /**
     * Creates a new LimelightState with the given parameters.
     * @param limelightPipeline the pipeline that the limelight should be running.
     */
    public LimelightState(LimelightPipeline limelightPipeline) {
        this(true, limelightPipeline);
    }


    /**
     * @return whether the Limelight is enabled in this state.
     */
    public boolean getEnabled() {
        return enabled;
    }

    /**
     * @return the LimelightPipeline in this state.
     */
    public LimelightPipeline getPipeline() {
        return limelightPipeline;
    }

    @Override
    public String toString() {
        return String.format("enabled: %s, limelightPipeline.pipelineIndex: %s", enabled, limelightPipeline.pipelineIndex);
    }

    @Override
    public String getDisplayName() {
        return "LimelightState";
    }
}
