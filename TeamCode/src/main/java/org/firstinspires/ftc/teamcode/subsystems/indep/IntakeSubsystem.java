package org.firstinspires.ftc.teamcode.subsystems.indep;

import org.firstinspires.ftc.teamcode.HardwareRobot;

public class IntakeSubsystem extends SubInDepSubsystem<
        IntakeSubsystem.State,
        IntakeSubsystem.SemidirectControlData,
        IntakeSubsystem.DirectControlData
> {
    private HardwareRobot robot;
    private State state;
    private PositionTargetData targetData;

    public IntakeSubsystem(HardwareRobot robot) {
        super();
        this.robot = robot;
        this.state = State.REST;
        this.targetData = state.data;
    }

    public enum State {
        //tune these
        APPROACH_O(0, 0, 0, 0),
        INTAKE_O(0, 0, 0, 0),
        INTAKE_C(0, 0, 0, 0),
        APPROACH_C(0, 0, 0, 0),
        TRANSFER_C(0, 0, 0, 0),
        TRANSFER_O(0, 0, 0, 0),
        REST(0, 0, 0, 0);

        private final PositionTargetData data;

        State(double intakePivotPos, double intakeWristPos, double intakeClawPos, double intakeSlidePos){
            data = new PositionTargetData();
            data.intakePivotPos = intakePivotPos;
            data.intakeWristPos = intakeWristPos;
            data.intakeClawPos = intakeClawPos;
            data.intakeLeftSlidePos = intakeSlidePos;
            data.intakeRightSlidePos = intakeSlidePos;
        }
    }

    public static class PositionTargetData {
        private double
                intakePivotPos,
                intakeWristPos,
                intakeClawPos,
                intakeLeftSlidePos,
                intakeRightSlidePos;
    }

    public static class DirectControlData {
        public double intakeClawPower;
        public double intakeWristPower;
        public double intakePivotPower;
        public double intakeLeftLiftPower;
        public double intakeRightLiftPower;
    }

    public static class SemidirectControlData {
        public double intakeClawPower;
        public double intakeWristPower;
        public double intakePivotPower;
        public double intakeLiftPower;
    }

    public void semidirectControl(SemidirectControlData data) {
        targetData.intakeClawPos = InDepSubsystem.clamp(
                targetData.intakeClawPos + data.intakeClawPower, 0, 1
        );  // update clamp bounds later
        targetData.intakePivotPos = InDepSubsystem.clamp(
                targetData.intakePivotPos + data.intakePivotPower, 0, 1
        );
        targetData.intakeWristPos = InDepSubsystem.clamp(
                targetData.intakeWristPos + data.intakeWristPower, 0, 1
        );
        targetData.intakeLeftSlidePos = InDepSubsystem.clamp(
                targetData.intakeLeftSlidePos
                        + 0.5*(InDepSubsystem.sigmoid(20*data.intakeLiftPower-10)+0.5),
                0, 1
        );
        targetData.intakeRightSlidePos = targetData.intakeLeftSlidePos;
    }

    public void directControl(DirectControlData data) {
        targetData.intakeClawPos += data.intakeClawPower;
        targetData.intakePivotPos += data.intakePivotPower;
        targetData.intakeWristPos += data.intakeWristPower;
        targetData.intakeLeftSlidePos += data.intakeLeftLiftPower;
        targetData.intakeRightSlidePos += data.intakeRightLiftPower;
    }

    public void setState(State state) {
        this.state = state;
        this.targetData = state.data;

        robot.intakeClaw.setPosition(targetData.intakeClawPos);
        robot.intakeWrist.setPosition(targetData.intakeWristPos);
        robot.intakePivot.setPosition(targetData.intakePivotPos);
        robot.leftIntakeLift.setPosition(targetData.intakeLeftSlidePos);
        robot.rightIntakeLift.setPosition(targetData.intakeRightSlidePos);
    }

    public State getState() {
        return state;
    }

    public boolean jobFulfilled() {
        return (
            approxEq(robot.intakeClaw.getPosition(), targetData.intakeClawPos) &&
            approxEq(robot.intakeWrist.getPosition(), targetData.intakeWristPos) &&
            approxEq(robot.intakePivot.getPosition(), targetData.intakePivotPos) &&
            approxEq(robot.leftIntakeLift.getPosition(), targetData.intakeLeftSlidePos) &&
            approxEq(robot.rightIntakeLift.getPosition(), targetData.intakeRightSlidePos)
        );
    }

}
