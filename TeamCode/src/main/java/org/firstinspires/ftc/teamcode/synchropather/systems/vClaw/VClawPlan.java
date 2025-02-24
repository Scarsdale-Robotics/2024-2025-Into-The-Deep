package org.firstinspires.ftc.teamcode.synchropather.systems.vClaw;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.synchropather.subsystemclasses.HorizontalIntakeSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.subsystemclasses.VerticalDepositSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.systems.MovementType;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.Movement;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.Plan;

/**
 * Contains a sequence of vertical claw Movements.
 */
public class VClawPlan extends Plan<VClawState> {

    private final VerticalDepositSubsystem verticalIntake;

    private final Telemetry telemetry;


    public VClawPlan(VerticalDepositSubsystem verticalIntake, Telemetry telemetry, Movement... movements) {
        super(MovementType.VERTICAL_CLAW, movements);
        this.verticalIntake = verticalIntake;
        this.telemetry = telemetry;
        if (telemetry != null) {
            telemetry.addData("[SYNCHROPATHER] HClawPlan desiredState.getPosition()", "-");
            telemetry.update();
        }
    }

    public VClawPlan(VerticalDepositSubsystem verticalIntake, Movement... movements) {
        this(verticalIntake, null, movements);
    }

    public void loop() {
        // Desired states
        VClawState desiredState = getCurrentState();

        verticalIntake.setClawPosition(desiredState.getPosition());

        if (telemetry != null) {
            telemetry.addData("[SYNCHROPATHER] HClawPlan desiredState.getPosition()", desiredState.getPosition());
            telemetry.update();
        }
    }

    @Override
    public void stop() {}

}
