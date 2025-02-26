package org.firstinspires.ftc.teamcode.synchropather.systems.vClaw;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.synchropather.subsystemclasses.VerticalDepositSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.systems.MovementType;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.Movement;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.Plan;

/**
 * Contains a sequence of vertical claw Movements.
 */
public class VClawPlan extends Plan<VClawState> {

    private final VerticalDepositSubsystem verticalDeposit;

    private final Telemetry telemetry;


    public VClawPlan(VerticalDepositSubsystem verticalDeposit, Telemetry telemetry, Movement... movements) {
        super(MovementType.VERTICAL_CLAW, movements);
        this.verticalDeposit = verticalDeposit;
        this.telemetry = telemetry;
        if (telemetry != null) {
            telemetry.addData("[SYNCHROPATHER] VClawPlan desiredState.getPosition()", "-");
            telemetry.update();
        }
    }

    public VClawPlan(VerticalDepositSubsystem verticalDeposit, Movement... movements) {
        this(verticalDeposit, null, movements);
    }

    public void loop() {
        // Desired states
        VClawState desiredState = getCurrentState();

        verticalDeposit.setClawPosition(desiredState.getPosition());

        if (telemetry != null) {
            telemetry.addData("[SYNCHROPATHER] VClawPlan desiredState.getPosition()", desiredState.getPosition());
            telemetry.update();
        }
    }

    @Override
    public void stop() {}

}
