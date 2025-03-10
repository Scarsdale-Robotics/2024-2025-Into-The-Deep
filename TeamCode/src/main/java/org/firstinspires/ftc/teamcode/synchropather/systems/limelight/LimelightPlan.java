package org.firstinspires.ftc.teamcode.synchropather.systems.limelight;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.teamcode.synchropather.subsystemclasses.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.systems.MovementType;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.Movement;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.Plan;

@Config
public class LimelightPlan extends Plan<LimelightState> {

    private final LimelightSubsystem limelightSubsystem;

    private LimelightState lastState;


    /**
     * Creates a new LimelightPlan with the given parameters
     * @param movements
     */
    public LimelightPlan(LimelightSubsystem limelightSubsystem, Movement... movements) {
        super(MovementType.LIMELIGHT, movements);
        this.limelightSubsystem = limelightSubsystem;
        this.lastState = null;
    }

    @Override
    public void loop() {
        LimelightState currentState = getCurrentState();

        // Check if any state components have changed since last loop
        boolean enabledChanged = false;
        boolean pipelineChanged = false;
        if (lastState == null) {
            enabledChanged = true;
            pipelineChanged = true;
        } else {
            if (lastState.getEnabled() != currentState.getEnabled()) {
                enabledChanged = true;
            }
            if (!lastState.getPipeline().equals(currentState.getPipeline())) {
                pipelineChanged = true;
            }
        }

        // Handle enabling/disabling
        if (enabledChanged) {
            if (currentState.getEnabled()) {
                limelightSubsystem.startLimelight();
            } else {
                limelightSubsystem.pauseLimelight();
            }
        }

        // Handle pipeline switching
        if (pipelineChanged) {
            limelightSubsystem.setCurrentState(currentState);
            limelightSubsystem.switchLimelightPipeline(currentState.getPipeline().pipelineIndex);
        }

        lastState = currentState;
    }

    @Override
    public void stop() {
        limelightSubsystem.pauseLimelight();
    }

}
