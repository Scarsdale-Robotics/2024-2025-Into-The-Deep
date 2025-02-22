package org.firstinspires.ftc.teamcode.synchropather.systems.limelight;

public enum LimelightPipeline {
    NONE(-1),
    APRIL_TAG(0),
    SAMPLE_DETECTOR(2);

    public final int pipelineIndex;

    LimelightPipeline(int pipelineIndex) {
        this.pipelineIndex = pipelineIndex;
    }
}
