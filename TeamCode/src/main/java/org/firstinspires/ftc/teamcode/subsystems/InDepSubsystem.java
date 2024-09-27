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

    // TODO: add a way to vary the speed at which an item executes
    public void executeTasks() {
        while (OP_MODE.opModeIsActive()) {
            InDepTask currentTask = getCurrentTask();
            if (currentTask == null) continue;

            setClawUDPosition(currentTask.CLAW_UD_POSITION);
            setClawOCPosition(currentTask.CLAW_OC_POSITION);
            setElbowPosition(currentTask.ELBOW_POSITION);
            setLiftPosition(currentTask.LIFT_POSITION);

            if (
                HARDWARE_ROBOT.clawUD.getPosition() == currentTask.CLAW_UD_POSITION.SERVO_POSITION &&
                HARDWARE_ROBOT.clawOC.getPosition() == currentTask.CLAW_OC_POSITION.SERVO_POSITION &&
                HARDWARE_ROBOT.elbow.getPosition() == currentTask.ELBOW_POSITION.SERVO_POSITION &&
                HARDWARE_ROBOT.lift.atTargetPosition()
            ) {
                completeCurrentTask();
            }
        }
    }

    ///////////////
    // CLAW (OC) //
    ///////////////
    public enum ClawOCPosition {
        CLOSED(0),
        PARTIAL(0.5),
        OPEN(1);

        public final double SERVO_POSITION;
        private ClawOCPosition(double servoPosition) {
            SERVO_POSITION = servoPosition;
        }
    }
    public void setClawOCPosition(ClawOCPosition position) {
        setClawOCPosition(position.SERVO_POSITION);
    }
    public void setClawOCPosition(double position) {
        HARDWARE_ROBOT.clawOC.setPosition(position);
    }

    ///////////////
    // CLAW (UD) //
    ///////////////
    public enum ClawUDPosition {
        DOWN(0),
        CENTER(0.5),
        UP(1);

        public final double SERVO_POSITION;

        private ClawUDPosition(double servoPosition) {
            SERVO_POSITION = servoPosition;
        }
    }
    public void setClawUDPosition(ClawUDPosition position) {
        setClawUDPosition(position.SERVO_POSITION);
    }
    public void setClawUDPosition(double position) {
        HARDWARE_ROBOT.clawUD.setPosition(position);
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
    }

    //////////////////////////
    // GENERAL IN-DEP TASKS //
    //////////////////////////
    private final ArrayList<InDepTask> TASK_LIST = new ArrayList<>(
        Collections.singletonList(InDepTask.INIT)
    );
    public enum InDepTask {
        INIT(ClawUDPosition.UP, ClawOCPosition.CLOSED, ElbowPosition.UP, LiftPosition.LOW),
        PRE_ABV_INTAKE(ClawUDPosition.DOWN, ClawOCPosition.CLOSED, ElbowPosition.CENTER, LiftPosition.LOW),
        PST_ABV_INTAKE(ClawUDPosition.DOWN, ClawOCPosition.OPEN, ElbowPosition.CENTER, LiftPosition.LOW),
        PRE_ARD_INTAKE(ClawUDPosition.DOWN, ClawOCPosition.OPEN, ElbowPosition.CENTER, LiftPosition.LOW),
        PST_ARD_INTAKE(ClawUDPosition.DOWN, ClawOCPosition.CLOSED, ElbowPosition.CENTER, LiftPosition.LOW),
        ENTER_SUB(ClawUDPosition.CENTER, ClawOCPosition.CLOSED, ElbowPosition.CENTER, LiftPosition.LOW),
        EXIT_SUB(ClawUDPosition.CENTER, ClawOCPosition.CLOSED, ElbowPosition.CENTER, LiftPosition.LOW),
        PRE_DEP_HP(ClawUDPosition.DOWN, ClawOCPosition.CLOSED, ElbowPosition.CENTER, LiftPosition.LOW),
        PST_DEP_HP(ClawUDPosition.DOWN, ClawOCPosition.OPEN, ElbowPosition.CENTER, LiftPosition.LOW),
        PRE_DEP_SPC(ClawUDPosition.CENTER, ClawOCPosition.CLOSED, ElbowPosition.CENTER, LiftPosition.ABV_SPC),
        PST_DEP_SPC(ClawUDPosition.CENTER, ClawOCPosition.PARTIAL, ElbowPosition.CENTER, LiftPosition.BLW_SPC),
        PRE_DEP_BSK(ClawUDPosition.CENTER, ClawOCPosition.CLOSED, ElbowPosition.UPPER_CENTER, LiftPosition.BSK),
        PST_DEP_BSK(ClawUDPosition.CENTER, ClawOCPosition.CLOSED, ElbowPosition.UPPER_CENTER, LiftPosition.BSK),
        HANG(ClawUDPosition.UP, ClawOCPosition.CLOSED, ElbowPosition.UP, LiftPosition.HANG);

        public final ClawUDPosition CLAW_UD_POSITION;
        public final ClawOCPosition CLAW_OC_POSITION;
        public final ElbowPosition ELBOW_POSITION;
        public final LiftPosition LIFT_POSITION;
        private InDepTask(
            ClawUDPosition clawUDPosition,
            ClawOCPosition clawOCPosition,
            ElbowPosition elbowPosition,
            LiftPosition liftPosition
        ) {
            CLAW_UD_POSITION = clawUDPosition;
            CLAW_OC_POSITION = clawOCPosition;
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
