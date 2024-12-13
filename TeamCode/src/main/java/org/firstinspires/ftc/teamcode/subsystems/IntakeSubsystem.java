package org.firstinspires.ftc.teamcode.subsystems;

public class IntakeSubsystem {

    public IntakeSubsystem() {

    }

    enum State {
        SUB_APRCH(0, 0, 0),
        SUB_INTK(0, 0, 0);

        private final double intakePivotPos, intakeWristPos, intakeClawPos;

        State(double intakePivotPos, double intakeWristPos, double intakeClawPos){
            this.intakePivotPos = intakePivotPos;
            this.intakeWristPos = intakeWristPos;
            this.intakeClawPos = intakeClawPos;
        }
    }

}
