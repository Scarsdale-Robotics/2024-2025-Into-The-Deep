package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareRobot;

import java.util.ArrayList;
import java.util.Collections;

public class InDepSubsystem extends SubsystemBase {

    private final HardwareRobot HARDWARE_ROBOT;
    private final LinearOpMode OP_MODE;

    public InDepSubsystem(HardwareRobot hardwareRobot, LinearOpMode opMode) {
        HARDWARE_ROBOT = hardwareRobot;
        OP_MODE = opMode;

        executeTasks();
    }

    // TODO: add a way to vary the speed at which a task executes
    public void executeTasks() {
        while (OP_MODE.opModeIsActive()) {
            InDepTask currentTask = getCurrentTask();
            if (currentTask == null) continue;

            setClawPosition(currentTask.CLAW_POSITION);
            setElbowPosition(currentTask.ELBOW_POSITION);
            setLiftPosition(currentTask.LIFT_POSITION);

            if (
                HARDWARE_ROBOT.claw.getPosition() == currentTask.CLAW_POSITION.SERVO_POSITION &&
                HARDWARE_ROBOT.elbow.getPosition() == currentTask.ELBOW_POSITION.SERVO_POSITION &&
                HARDWARE_ROBOT.lift.atTargetPosition()
            ) {
                completeCurrentTask();
            }
        }
    }

    //////////
    // CLAW //
    //////////
    // TODO: Tune all servo values
    public enum ClawPosition {
        CLOSED(0),
        PARTIAL(0.5),
        OPEN(1);

        public final double SERVO_POSITION;
        private ClawPosition(double servoPosition) {
            SERVO_POSITION = servoPosition;
        }
    }
    public void setClawPosition(ClawPosition position) {
        setClawPosition(position.SERVO_POSITION);
    }
    public void setClawPosition(double position) {
        HARDWARE_ROBOT.claw.setPosition(position);
    }

    ///////////
    // ELBOW //
    ///////////
    public enum ElbowPosition {
        CENTER(0.5),
        UPPER_CENTER(0.75),
        UP(1);

        public final double SERVO_POSITION;

        private ElbowPosition(double servoPosition) {
            SERVO_POSITION = servoPosition;
        }
    }
    public void setElbowPosition(ElbowPosition position) {
        setElbowPosition(position.SERVO_POSITION);
    }
    public void setElbowPosition(double position) {
        HARDWARE_ROBOT.elbow.setPosition(position);
    }

    //////////
    // LIFT //
    //////////
    public enum LiftPosition {
        LOW(0),
        ABV_SPC(5000),
        BLW_SPC(4000),
        BSK(7000),
        HANG(2500);

        public final int ENCODER_TICKS;
        private LiftPosition(int encoderTicks) {
            ENCODER_TICKS = encoderTicks;
        }
    }
    public void setLiftPosition(LiftPosition position) {
        setLiftPosition(position.ENCODER_TICKS);
    }

    public void setLiftPosition(int position) {
        HARDWARE_ROBOT.lift.setTargetPosition(position);
        // ew, use synchropather liftplan pid instead

    }
    public int getLiftPosition() {
        return HARDWARE_ROBOT.lift.getCurrentPosition();
    }
    public void setLiftPower(double power) {
        HARDWARE_ROBOT.lift.motor.setPower(power);
    }


    //////////////////////////
    // GENERAL IN-DEP TASKS //
    //////////////////////////
    private final ArrayList<InDepTask> TASK_LIST = new ArrayList<>(
        Collections.singletonList(InDepTask.INIT)
    );
    public enum InDepTask {
        INIT(ClawPosition.CLOSED, ElbowPosition.UP, LiftPosition.LOW),

        // ABV_INTAKE: When gripping the sample from above by inserting our claw into the sample's
        //             triangular prism-shaped inset and pushing "out"
        PRE_ABV_INTAKE(ClawPosition.CLOSED, ElbowPosition.CENTER, LiftPosition.LOW),
        PST_ABV_INTAKE(ClawPosition.OPEN, ElbowPosition.CENTER, LiftPosition.LOW),

        // ARD_INTAKE: The preferred intake method when possible (unless engineering says otherwise),
        //             involves wrapping the claw's grippers around the sample's two sides that are
        //             the most perpendicular to the ground
        PRE_ARD_INTAKE(ClawPosition.OPEN, ElbowPosition.CENTER, LiftPosition.LOW),
        PST_ARD_INTAKE(ClawPosition.CLOSED, ElbowPosition.CENTER, LiftPosition.LOW),

        // ENTER_SUB, EXIT_SUB: Entering and exiting the submersible
        ENTER_SUB(ClawPosition.CLOSED, ElbowPosition.CENTER, LiftPosition.LOW),
        EXIT_SUB(ClawPosition.CLOSED, ElbowPosition.CENTER, LiftPosition.LOW),

        // DEP_HP: Deposit to the human player (in the observation zone)
        PRE_DEP_HP(ClawPosition.CLOSED, ElbowPosition.CENTER, LiftPosition.LOW),
        PST_DEP_HP(ClawPosition.OPEN, ElbowPosition.CENTER, LiftPosition.LOW),

        // DEP_SPC: Deposit on the high chamber (specimen)
        PRE_DEP_SPC(ClawPosition.CLOSED, ElbowPosition.UP, LiftPosition.ABV_SPC),
        PST_DEP_SPC(ClawPosition.PARTIAL, ElbowPosition.UP, LiftPosition.BLW_SPC),

        // DEP_BSK: Deposit into the high basket (sample)
        PRE_DEP_BSK(ClawPosition.CLOSED, ElbowPosition.UPPER_CENTER, LiftPosition.BSK),
        PST_DEP_BSK(ClawPosition.CLOSED, ElbowPosition.UPPER_CENTER, LiftPosition.BSK),

        LOW_HANG(ClawPosition.CLOSED, ElbowPosition.UP, LiftPosition.HANG);

        public final ClawPosition CLAW_POSITION;
        public final ElbowPosition ELBOW_POSITION;
        public final LiftPosition LIFT_POSITION;
        private InDepTask(
            ClawPosition clawPosition,
            ElbowPosition elbowPosition,
            LiftPosition liftPosition
        ) {
            CLAW_POSITION = clawPosition;
            ELBOW_POSITION = elbowPosition;
            LIFT_POSITION = liftPosition;
        }
    }
    public InDepTask getCurrentTask() {
        return TASK_LIST.isEmpty() ? null : TASK_LIST.get(0);
    }
    public enum TaskInsertionPosition {
        CURRENT(0),
        NEXT(1),
        LAST(-1);

        public final int ARRAY_IX;
        private TaskInsertionPosition(int arrayIX) {
            ARRAY_IX = arrayIX;
        }
    }
    public void addTask(InDepTask task, TaskInsertionPosition position) {
        TASK_LIST.add(
            position.ARRAY_IX < 0
                ? TASK_LIST.size() + position.ARRAY_IX
                : position.ARRAY_IX,
            task
        );
    }
    public void completeCurrentTask() {
        skipCurrentTask();
    }
    public void skipCurrentTask() {
        TASK_LIST.remove(0);
    }
    public void clearTasks(boolean cancelCurrent) {
        for (int i = (cancelCurrent ? 1 : 0); i< TASK_LIST.size(); i++) {
            TASK_LIST.remove(0);
        }
    }

}
