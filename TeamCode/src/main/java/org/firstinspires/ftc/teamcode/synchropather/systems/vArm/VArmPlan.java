package org.firstinspires.ftc.teamcode.synchropather.systems.vArm;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.synchropather.subsystemclasses.HorizontalIntakeSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.subsystemclasses.VerticalDepositSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.systems.MovementType;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.Movement;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.Plan;

/**
 * Contains a sequence of vertical arm Movements.
 */
public class VArmPlan extends Plan<VArmState> {

    private final VerticalDepositSubsystem verticalDeposit;

    private final Telemetry telemetry;


    public VArmPlan(VerticalDepositSubsystem verticalDeposit, Telemetry telemetry, Movement... movements) {
        super(MovementType.VERTICAL_ARM, movements);
        this.verticalDeposit = verticalDeposit;
        this.telemetry = telemetry;
        if (telemetry != null) {
            telemetry.addData("[SYNCHROPATHER] HArmPlan desiredState.getPosition()", "-");
            telemetry.update();
        }
    }

    public VArmPlan(VerticalDepositSubsystem verticalDeposit, Movement... movements) {
        this(verticalDeposit, null, movements);
    }

    public void loop() {
        // Desired states
        VArmState desiredState = getCurrentState();

        verticalDeposit.setArmPosition(desiredState.getPosition());

        if (telemetry != null) {
            telemetry.addData("[SYNCHROPATHER] HArmPlan desiredState.getPosition()", desiredState.getPosition());
            telemetry.update();
        }
    }

    @Override
    public void stop() {}

}
