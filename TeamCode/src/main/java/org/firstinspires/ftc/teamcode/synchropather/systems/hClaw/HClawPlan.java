package org.firstinspires.ftc.teamcode.synchropather.systems.hClaw;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.synchropather.subsystemclasses.HorizontalIntakeSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.systems.MovementType;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.Movement;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.Plan;

/**
 * Contains a sequence of horizontal claw Movements.
 */
public class HClawPlan extends Plan<HClawState> {

    private final HorizontalIntakeSubsystem horizontalIntake;

    private final Telemetry telemetry;


    public HClawPlan(HorizontalIntakeSubsystem horizontalIntake, Telemetry telemetry, Movement... movements) {
        super(MovementType.HORIZONTAL_CLAW, movements);
        this.horizontalIntake = horizontalIntake;
        this.telemetry = telemetry;
        if (telemetry != null) {
            telemetry.addData("[SYNCHROPATHER] HClawPlan desiredState.getPosition()", "-");
            telemetry.update();
        }
    }

    public HClawPlan(HorizontalIntakeSubsystem horizontalIntake, Movement... movements) {
        this(horizontalIntake, null, movements);
    }

    public void loop() {
        // Desired states
        HClawState desiredState = getCurrentState();

        horizontalIntake.setClawPosition(desiredState.getPosition());

        if (telemetry != null) {
            telemetry.addData("[SYNCHROPATHER] HClawPlan desiredState.getPosition()", desiredState.getPosition());
            telemetry.update();
        }
    }

    @Override
    public void stop() {}

}
