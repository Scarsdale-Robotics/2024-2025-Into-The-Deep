package org.firstinspires.ftc.teamcode.synchropather.systems.hArm;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.synchropather.subsystemclasses.HorizontalIntakeSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.systems.MovementType;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.Movement;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.Plan;

/**
 * Contains a sequence of horizontal arm Movements.
 */
public class HArmPlan extends Plan<HArmState> {

    private final HorizontalIntakeSubsystem horizontalIntake;

    private final Telemetry telemetry;


    public HArmPlan(HorizontalIntakeSubsystem horizontalIntake, Telemetry telemetry, Movement... movements) {
        super(MovementType.HORIZONTAL_ARM, movements);
        this.horizontalIntake = horizontalIntake;
        this.telemetry = telemetry;
        if (telemetry != null) {
            telemetry.addData("[SYNCHROPATHER] HArmPlan desiredState.getPosition()", "-");
            telemetry.update();
        }
    }

    public HArmPlan(HorizontalIntakeSubsystem horizontalIntake, Movement... movements) {
        this(horizontalIntake, null, movements);
    }

    public void loop() {
        // Desired states
        HArmState desiredState = getCurrentState();

        horizontalIntake.setArmPosition(desiredState.getPosition());

        if (telemetry != null) {
            telemetry.addData("[SYNCHROPATHER] HArmPlan desiredState.getPosition()", desiredState.getPosition());
            telemetry.update();
        }
    }

    @Override
    public void stop() {}

}
