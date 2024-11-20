package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareRobot;
import org.opencv.core.Point;

import java.util.ArrayList;
import java.util.Collections;

public class InDepSubsystem extends SubsystemBase {

    private final HardwareRobot HARDWARE_ROBOT;
    private final LinearOpMode OP_MODE;
    private final DriveSubsystem DRIVE;
    private final CVSubsystem CV;

    public final double ABV_INTAKE_ALIGN_TGT_X = 0.5;  // percentage of width
    public final double ABV_INTAKE_ALIGN_TGT_Y = 0.5;  // percentage of height
    public final double ARD_INTAKE_ALIGN_TGT_X = 0.5;  // percentage of width
    public final double ARD_INTAKE_ALIGN_TGT_Y = 0.5;  // percentage of height

    public final int ABV_INTAKE_ALIGN_ERR_THRESH_X = 6;  // in pixels
    public final int ABV_INTAKE_ALIGN_ERR_THRESH_Y = 6;  // in pixels
    public final int ARD_INTAKE_ALIGN_ERR_THRESH_X = 30;  // in pixels
    public final int ARD_INTAKE_ALIGN_ERR_THRESH_Y = 30;  // in pixels

    public InDepSubsystem(
            HardwareRobot hardwareRobot,
            LinearOpMode opMode,
            DriveSubsystem driveSubsystem,
            CVSubsystem cvSubsystem
    ) {
        HARDWARE_ROBOT = hardwareRobot;
        OP_MODE = opMode;
        DRIVE = driveSubsystem;
        CV = cvSubsystem;
    }

    public void executeTasks() {
        InDepTask currentTask = getCurrentTask();
        if (currentTask == null) return;

        setClawPosition(currentTask.CLAW_POSITION);
        setElbowPosition(currentTask.ELBOW_POSITION);
        setLiftPosition(currentTask.LIFT_POSITION);

        // alignment and other task-specific commands
        boolean taskPossiblyComplete = false;
        Point error;
        switch (currentTask) {
            case PRE_ABV_INTAKE:
                error = CV.getSampleOffset();
                error.x -= CV.CAM_SZ.getWidth() * ABV_INTAKE_ALIGN_TGT_X;
                error.y -= CV.CAM_SZ.getHeight() * ABV_INTAKE_ALIGN_TGT_Y;
                DRIVE.stepTowards2DPoint(error);
                if (
                        Math.abs(error.x) <= ABV_INTAKE_ALIGN_ERR_THRESH_X &&
                        Math.abs(error.y) <= ABV_INTAKE_ALIGN_ERR_THRESH_Y
                ) {
                    taskPossiblyComplete = true;
                }
                break;
            case PRE_ARD_INTAKE:
                error = CV.getSampleOffset();
                error.x -= CV.CAM_SZ.getWidth() * ARD_INTAKE_ALIGN_TGT_X;
                error.y -= CV.CAM_SZ.getHeight() * ARD_INTAKE_ALIGN_TGT_Y;
                DRIVE.stepTowards2DPoint(error);
                if (
                        Math.abs(error.x) <= ARD_INTAKE_ALIGN_ERR_THRESH_X &&
                        Math.abs(error.y) <= ARD_INTAKE_ALIGN_ERR_THRESH_Y
                ) {
                    taskPossiblyComplete = true;
                }
                break;
        }

        if (
            HARDWARE_ROBOT.claw.getPosition() == currentTask.CLAW_POSITION.SERVO_POSITION &&
            HARDWARE_ROBOT.elbow.getPosition() == currentTask.ELBOW_POSITION.SERVO_POSITION &&
            HARDWARE_ROBOT.lift.atTargetPosition() && taskPossiblyComplete
        ) {
            // Update tasks
            switch (currentTask) {
                case PRE_ABV_INTAKE:
                    addTask(InDepTask.PST_ABV_INTAKE, TaskInsertionPosition.NEXT);
                    break;
                case PRE_ARD_INTAKE:
                    addTask(InDepTask.PRE_ARD_INTAKE, TaskInsertionPosition.NEXT);
                    break;
            }

            completeCurrentTask();
        }
    }

    //////////
    // CLAW //
    //////////
    // TODO: Tune all servo values
    public enum ClawPosition {
        CLOSED(0, "Closed"),
        PARTIAL(0.5, "Partial"),
        OPEN(1, "Open");

        public final double SERVO_POSITION;
        public final String NAME;
        private ClawPosition(double servoPosition, String name) {
            SERVO_POSITION = servoPosition;
            NAME = name;
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
        CENTER(0.5, "Center"),
        LOWER_UPPER_CENTER(0.6, "Lower upper center"),
        UPPER_CENTER(0.75, "Upper center"),
        UP(1, "Up");

        public final double SERVO_POSITION;
        public final String NAME;

        private ElbowPosition(double servoPosition, String name) {
            SERVO_POSITION = servoPosition;
            NAME = name;
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
        LiftPosition(int encoderTicks) {
            ENCODER_TICKS = encoderTicks;
        }
    }
    public void setLiftPosition(LiftPosition position) {
        setLiftPosition(position.ENCODER_TICKS);
    }

    public void setLiftPosition(int position) {
        HARDWARE_ROBOT.leftLift.setTargetPosition(position);
        // ew, use synchropather liftplan pid instead

    }
    public int getLeftLiftPosition() {
        return HARDWARE_ROBOT.leftLift.getCurrentPosition();
    }
    public int getRightLiftPosition() {
        return HARDWARE_ROBOT.rightLift.getCurrentPosition();
    }

    public void setLeftLiftPower(double power) {
        HARDWARE_ROBOT.leftLift.motor.setPower(power);
    }

    public void setRightLiftPower(double power) {
        HARDWARE_ROBOT.rightLift.motor.setPower(power);
    }


    /////////////////////
    // TASK MANAGEMENT //
    /////////////////////
    private final ArrayList<InDepTask> TASK_LIST = new ArrayList<>(
        Collections.singletonList(InDepTask.INIT)
    );
    public enum InDepTask {
        INIT(ClawPosition.CLOSED, ElbowPosition.UP, LiftPosition.LOW),

        // ABV_INTAKE: When gripping the sample from above by inserting our claw into the sample's
        //             triangular prism-shaped inset and pushing "out"
        PRE_ABV_INTAKE(ClawPosition.CLOSED, ElbowPosition.LOWER_UPPER_CENTER, LiftPosition.LOW),
        PST_ABV_INTAKE(ClawPosition.OPEN, ElbowPosition.LOWER_UPPER_CENTER, LiftPosition.LOW),

        // ARD_INTAKE: The preferred intake method when possible (unless engineering says otherwise),
        //             involves wrapping the claw's grippers around the sample's two sides that are
        //             the most perpendicular to the ground
        PRE_ARD_INTAKE(ClawPosition.OPEN, ElbowPosition.LOWER_UPPER_CENTER, LiftPosition.LOW),
        PST_ARD_INTAKE(ClawPosition.CLOSED, ElbowPosition.CENTER, LiftPosition.LOW),

        // ENTER_SUB, EXIT_SUB: Entering and exiting the submersible
        ENTER_SUB(ClawPosition.CLOSED, ElbowPosition.LOWER_UPPER_CENTER, LiftPosition.LOW),
        EXIT_SUB(ClawPosition.CLOSED, ElbowPosition.LOWER_UPPER_CENTER, LiftPosition.LOW),

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
        InDepTask(
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
        TaskInsertionPosition(int arrayIX) {
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
