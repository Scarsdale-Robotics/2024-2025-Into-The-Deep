package org.firstinspires.ftc.teamcode.subsystems.indep;

import org.firstinspires.ftc.teamcode.HardwareRobot;

public class IntakeSubsystem extends SubInDepSubsystem<
        IntakeSubsystem.State,
        IntakeSubsystem.SemidirectControlData,
        IntakeSubsystem.DirectControlData
> {
    private final HardwareRobot robot;
    private State state;
    private PositionTargetData targetData;

    public IntakeSubsystem(HardwareRobot robot) {
        super();
        this.robot = robot;
        this.state = State.REST;
        this.targetData = state.data;
    }

    private static final int SKIP = -1000;

    public enum State {
        //tune these
        APPROACH_O(0, 0, 0, 0),
        INTAKE_O(0, 0, 0, SKIP),
        INTAKE_C(0, 0, 0, SKIP),
        APPROACH_C(0, 0, 0, SKIP),
        TRANSFER_C(0, 0, 0, 0),
        TRANSFER_O(0, 0, 0, 0),
        REST(0, 0, 0, 0);

        private final PositionTargetData data;

        State(double elbowPos, double wristPos, double clawPos, double intakeSlidePos){
            data = new PositionTargetData();
            data.elbowPos = elbowPos;
            data.wristPos = wristPos;
            data.clawPos = clawPos;
            data.extendoPos = intakeSlidePos;
        }
    }

    public static class PositionTargetData {
        private double
                elbowPos,
                wristPos,
                clawPos,
                extendoPos;
    }

    public static class DirectControlData {
        public double clawPower;
        public double wristPower;
        public double elbowPower;
        public double intakeLeftLiftPower;
        public double intakeRightLiftPower;
    }

    public static class SemidirectControlData {
        public double clawPower;
        public double wristPower;
        public double elbowPower;
        public double intakeLiftPower;
    }

    public void semidirectControl(SemidirectControlData data) {
        targetData.clawPos = InDepSubsystem.clamp(
                targetData.clawPos + data.clawPower, 0, 1
        );  // update clamp bounds later
        targetData.elbowPos = InDepSubsystem.clamp(
                targetData.elbowPos + data.elbowPower, 0, 1
        );
        targetData.wristPos = InDepSubsystem.clamp(
                targetData.wristPos + data.wristPower, 0, 1
        );
        targetData.extendoPos = InDepSubsystem.clamp(
                targetData.extendoPos
                        + 0.5*(InDepSubsystem.sigmoid(20*data.intakeLiftPower-10)+0.5),
                0, 1
        );
        targetData.intakeRightSlidePos = InDepSubsystem.clamp(
                targetData.intakeRightSlidePos
                        + 0.5*(InDepSubsystem.sigmoid(20*data.intakeLiftPower-10)+0.5),
                0, 1
        );
    }

    public void directControl(DirectControlData data) {
        targetData.clawPos += data.clawPower;
        targetData.elbowPos += data.elbowPower;
        targetData.wristPos += data.wristPower;
        targetData.extendoPos += data.intakeLeftLiftPower;
        targetData.intakeRightSlidePos += data.intakeRightLiftPower;
    }

    public void powerLift(double power) {
        this.targetData.intakeRightSlidePos += power;
        this.targetData.extendoPos += power;
    }

    public void setState(State state) {
        this.state = state;
        PositionTargetData pastTargetData = this.targetData;
        this.targetData = state.data;

        if (state.data.extendoPos == SKIP)
            this.targetData.extendoPos = pastTargetData.extendoPos;

        robot.claw.setPosition(targetData.clawPos);
        robot.wrist.setPosition(targetData.wristPos);
        robot.elbow.setPosition(targetData.elbowPos);

        if (targetData.extendoPos >= 0)
            robot.extendo.setTargetPosition(targetData.extendoPos);
    }

    public State getState() {
        return state;
    }

    public boolean jobFulfilled() {
        return (
            approxEq(robot.claw.getPosition(), targetData.clawPos) &&
            approxEq(robot.wrist.getPosition(), targetData.wristPos) &&
            approxEq(robot.elbow.getPosition(), targetData.elbowPos) &&
            approxEq(robot.extendo.getCurrentPosition(), targetData.extendoPos)
        );
    }

}
