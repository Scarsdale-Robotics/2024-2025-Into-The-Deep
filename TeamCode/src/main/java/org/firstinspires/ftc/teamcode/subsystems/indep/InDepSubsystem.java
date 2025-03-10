package org.firstinspires.ftc.teamcode.subsystems.indep;

import org.firstinspires.ftc.teamcode.HardwareRobot;
import org.firstinspires.ftc.teamcode.RobotSystem;

public class InDepSubsystem {
    // integrates intake, magazine, deposit, and transfer subsystems

    public RobotSystem robot;

    private final IntakeSubsystem intake;
    private final DepositSubsystem deposit;
    private final MagazineSubsystem mag;
    private final MakerSubsystem maker;
    private final ClipSubsystem clip;

    public enum ControlLaw {  // Not to be confused with Control Theory, which is insignificantly related
        // mechanical backup does not exist
        DIRECT,     // no FSM, no protections. drivers directly control motors/servos
        SEMIDIRECT, // no FSM, most basic protections remain (e.g. lift motors move together)
        ALTERNATE,  // only manual transitions in FSM
        NORMAL;     // automatic and manual transitions in FSM

        public boolean isGreaterThan(ControlLaw other) {
            return this.ordinal() > other.ordinal();
        }

        public boolean isLessThan(ControlLaw other) {
            return this.ordinal() < other.ordinal();
        }

        public boolean isEqualTo(ControlLaw other) {
            return this.ordinal() == other.ordinal();
        }

        public boolean isGreaterThanOrEqualTo(ControlLaw other) {
            return !isLessThan(other);
        }

        public boolean isLessThanOrEqualTo(ControlLaw other) {
            return !isGreaterThan(other);
        }

        public boolean isAtLeast(ControlLaw other) {
            return isGreaterThanOrEqualTo(other);
        }

        public boolean isAtMost(ControlLaw other) {
            return isLessThanOrEqualTo(other);
        }
    }

    private ControlLaw controlLaw;

    private final ControlLaw MIN_AUTOMATIC_TRANSITIONS_LAW = ControlLaw.NORMAL;
    private final ControlLaw MIN_MANUAL_TRANSITIONS_LAW = ControlLaw.ALTERNATE;

    public InDepSubsystem(RobotSystem robot, HardwareRobot hardware) {
        this.robot = robot;

        this.intake = new IntakeSubsystem(hardware);
        this.deposit = new DepositSubsystem(hardware);
        this.mag = new MagazineSubsystem(hardware);
        this.maker = new MakerSubsystem(hardware);
        this.clip = new ClipSubsystem(hardware);

        this.controlLaw = ControlLaw.NORMAL;

        this.lastControls = new InDepControlData();
    }

    public static class InDepControlData {
        // NORMAL, ALTERNATE
        public boolean intakeButton;
        public boolean depositButton;
        public boolean magButton;
        public boolean makerButton;
        public boolean clipButton;

        public boolean intakeUndo;
        public boolean depositUndo;
        public boolean magUndo;
        public boolean makerUndo;
        public boolean clipUndo;

        public double intakePower;
        public double depositPower;

        // SEMIDIRECT
        public IntakeSubsystem.SemidirectControlData intakeSemidirectCD;
        public DepositSubsystem.SemidirectControlData depositSemidirectCD;
        public MagazineSubsystem.SemidirectControlData magazineSemidirectCD;
        public MakerSubsystem.SemidirectControlData makerSemidirectCD;
        public ClipSubsystem.SemidirectControlData clipSemidirectCD;

        // DIRECT
        public IntakeSubsystem.DirectControlData intakeDirectCD;
        public DepositSubsystem.DirectControlData depositDirectCD;
        public MagazineSubsystem.DirectControlData magazineDirectCD;
        public MakerSubsystem.DirectControlData makerDirectCD;
        public ClipSubsystem.DirectControlData clipDirectCD;
    }

    private final InDepControlData lastControls;

    private IntakeSubsystem.State intakeState;
    private DepositSubsystem.State depositState;
    private MagazineSubsystem.State magState;
    private MakerSubsystem.State makerState;
    private ClipSubsystem.State clipState;

    public void setControlLaw(ControlLaw nextLaw) {
        controlLaw = nextLaw;
    }

    public ControlLaw getControlLaw() {
        return controlLaw;
    }

    public void tick(InDepControlData controls) {
        if (controlLaw.isEqualTo(ControlLaw.DIRECT)) tickDirect(controls);
        else if (controlLaw.isEqualTo(ControlLaw.SEMIDIRECT)) tickSemidirect(controls);
        else tickFSM(controls);

        lastControls.intakeButton = controls.intakeButton;
        lastControls.clipButton = controls.clipButton;
        lastControls.makerButton = controls.makerButton;
        lastControls.magButton = controls.magButton;
        lastControls.depositButton = controls.depositButton;

        lastControls.intakeUndo = controls.intakeUndo;
        lastControls.clipUndo = controls.clipUndo;
        lastControls.makerUndo = controls.makerUndo;
        lastControls.magUndo = controls.magUndo;
        lastControls.depositUndo = controls.depositUndo;

        lastControls.intakePower = controls.intakePower;
        lastControls.depositPower = controls.depositPower;

        // NOTE: SOME SUBSYSTEMS MAY REQUIRE LAST ACTION TRACKING.
        //       LAST ACTION TRACKING SHOULD BE IMPLEMENTED IMMEDIATELY BEFORE THIS COMMENT.
    }

    public void tickDirect(InDepControlData controls) {
        intake.directControl(controls.intakeDirectCD);
        deposit.directControl(controls.depositDirectCD);
        maker.directControl(controls.makerDirectCD);
        mag.directControl(controls.magazineDirectCD);
        clip.directControl(controls.clipDirectCD);
    }

    public void tickSemidirect(InDepControlData controls) {
        intake.semidirectControl(controls.intakeSemidirectCD);
        deposit.semidirectControl(controls.depositSemidirectCD);
        maker.semidirectControl(controls.makerSemidirectCD);
        mag.semidirectControl(controls.magazineSemidirectCD);
        clip.semidirectControl(controls.clipSemidirectCD);
    }

    private boolean intakeWasLastForward,
                    depositWasLastForward,
                    magWasLastForward,
                    makerWasLastForward,
                    clipWasLastForward;


    public void tickFSM(InDepControlData controls) {
        boolean
                fwdIntake = lastControls.intakeButton && !controls.intakeButton,
                fwdDeposit = lastControls.depositButton && !controls.depositButton,
                fwdMag = lastControls.magButton && !controls.magButton,
                fwdMaker = lastControls.makerButton && !controls.makerButton,
                fwdClip = lastControls.clipButton && !controls.clipButton,
                bwdIntake = lastControls.intakeUndo && !controls.intakeUndo,
                bwdDeposit = lastControls.depositUndo && !controls.depositUndo,
                bwdMag = lastControls.magUndo && !controls.magUndo,
                bwdMaker = lastControls.makerUndo && !controls.makerUndo,
                bwdClip = lastControls.clipUndo && !controls.clipUndo;

        if (fwdIntake) intakeWasLastForward = true;
        if (fwdDeposit) depositWasLastForward = true;
        if (fwdMag) magWasLastForward = true;
        if (fwdMaker) makerWasLastForward = true;
        if (fwdClip) clipWasLastForward = true;
        if (bwdIntake) intakeWasLastForward = false;
        if (bwdDeposit) depositWasLastForward = false;
        if (bwdMag) magWasLastForward = false;
        if (bwdMaker) makerWasLastForward = false;
        if (bwdClip) clipWasLastForward = false;


        // manual pass
        if (fwdIntake && controlLaw.isAtLeast(MIN_MANUAL_TRANSITIONS_LAW)) {
            switch (intake.getState()) {
                case REST:
                    intakeState = IntakeSubsystem.State.APPROACH_O;
                    break;
                case APPROACH_O:
                    intakeState = IntakeSubsystem.State.INTAKE_O;
                    break;
                case INTAKE_O:
                    intakeState = IntakeSubsystem.State.INTAKE_C;
                    break;
                case INTAKE_C:
                    intakeState = IntakeSubsystem.State.APPROACH_C;
                    break;
                case APPROACH_C:
                    intakeState = IntakeSubsystem.State.TRANSFER_C;
                    break;
                case TRANSFER_C:
                    intakeState = IntakeSubsystem.State.TRANSFER_O;
                    magState = MagazineSubsystem.State.DEQUEUE;
                    break;
                case TRANSFER_O:
                    intakeState = IntakeSubsystem.State.REST;
                    break;
            }
        }
        if (bwdIntake && controlLaw.isAtLeast(MIN_MANUAL_TRANSITIONS_LAW)){
            switch (intake.getState()) {
                case REST:
                    intakeState = IntakeSubsystem.State.TRANSFER_O;
                    break;
                case APPROACH_O:
                    intakeState = IntakeSubsystem.State.REST;
                    break;
                case INTAKE_O:
                    intakeState = IntakeSubsystem.State.APPROACH_O;
                    break;
                case INTAKE_C:
                    intakeState = IntakeSubsystem.State.INTAKE_O;
                    break;
                case APPROACH_C:
                    intakeState = IntakeSubsystem.State.INTAKE_C;
                    break;
                case TRANSFER_C:
                    intakeState = IntakeSubsystem.State.APPROACH_C;
                    break;
                case TRANSFER_O:
                    intakeState = IntakeSubsystem.State.TRANSFER_C;
                    break;
            }
        }
        if (intake.jobFulfilled() && controlLaw.isAtLeast(MIN_AUTOMATIC_TRANSITIONS_LAW)) {
            if (intakeWasLastForward) {
                switch (intake.getState()) {
                    case INTAKE_O:
                        intakeState = IntakeSubsystem.State.INTAKE_C;
                        break;
                    case INTAKE_C:
                        intakeState = IntakeSubsystem.State.APPROACH_C;
                        break;
                    case TRANSFER_C:
                        intakeState = IntakeSubsystem.State.TRANSFER_O;
                        magState = MagazineSubsystem.State.DEQUEUE;
                        break;
                    case TRANSFER_O:
                        intakeState = IntakeSubsystem.State.REST;
                        break;
                }
            } else {
                switch (intake.getState()) {
                    case INTAKE_C:
                        intakeState = IntakeSubsystem.State.INTAKE_O;
                        break;
                    case APPROACH_C:
                        intakeState = IntakeSubsystem.State.INTAKE_C;
                        break;
                    case TRANSFER_O:
                        intakeState = IntakeSubsystem.State.TRANSFER_C;
                        magState = MagazineSubsystem.State.REST;
                        break;
                    case REST:
                        intakeState = IntakeSubsystem.State.TRANSFER_O;
                        break;
                }
            }
        }

        //deposit manual
        if (fwdDeposit && controlLaw.isAtLeast(MIN_MANUAL_TRANSITIONS_LAW)) {
            switch (deposit.getState()) {
                case REST:
                    depositState = DepositSubsystem.State.TRANSFER_O;
                    break;
                case TRANSFER_O:
                    depositState = DepositSubsystem.State.TRANSFER_C;
                    break;
                case TRANSFER_C:
                    depositState = DepositSubsystem.State.APPROACH_C;
                    break;
                case APPROACH_C:
                    depositState = DepositSubsystem.State.DEPOSIT;
                    break;
                case DEPOSIT:
                    depositState = DepositSubsystem.State.REST;
                    break;
            }
        }
        if (bwdDeposit && controlLaw.isAtLeast(MIN_MANUAL_TRANSITIONS_LAW)) {
            switch (deposit.getState()) {
                case REST:
                    depositState = DepositSubsystem.State.DEPOSIT;
                    break;
                case TRANSFER_O:
                    depositState = DepositSubsystem.State.REST;
                    break;
                case TRANSFER_C:
                    depositState = DepositSubsystem.State.TRANSFER_O;
                    break;
                case APPROACH_C:
                    depositState = DepositSubsystem.State.TRANSFER_C;
                    break;
                case DEPOSIT:
                    depositState = DepositSubsystem.State.APPROACH_C;
                    break;
            }
        }
        //deposit auto
        if (deposit.jobFulfilled() && controlLaw.isAtLeast(MIN_AUTOMATIC_TRANSITIONS_LAW)) {
            if (depositWasLastForward) {
                switch (deposit.getState()) {
                    case TRANSFER_O:
                        depositState = DepositSubsystem.State.TRANSFER_C;
                        break;
                    case DEPOSIT:
                        depositState = DepositSubsystem.State.REST;
                        break;
                }
            } else {
                switch (deposit.getState()) {
                    case TRANSFER_C:
                        depositState = DepositSubsystem.State.TRANSFER_O;
                        break;
                    case REST:
                        depositState = DepositSubsystem.State.DEPOSIT;
                        break;
                }
            }
        }

        //magazine manual

        if (fwdMag && controlLaw.isAtLeast(MIN_MANUAL_TRANSITIONS_LAW)) {
            switch (mag.getState()) {
                case REST:
                    magState = MagazineSubsystem.State.DEQUEUE;
                    break;
                case DEQUEUE:
                    magState = MagazineSubsystem.State.REST;
                    break;
            }
        }
        if (bwdMag && controlLaw.isAtLeast(MIN_MANUAL_TRANSITIONS_LAW)) {
            switch (mag.getState()) {
                case REST:
                    magState = MagazineSubsystem.State.DEQUEUE;
                    break;
                case DEQUEUE:
                    magState = MagazineSubsystem.State.REST;
                    break;
            }
        }
        //magazine auto
        if (mag.jobFulfilled() && controlLaw.isAtLeast(MIN_AUTOMATIC_TRANSITIONS_LAW)) {
            if (magWasLastForward) {
                switch (mag.getState()) {  // for consistent formatting
                    case DEQUEUE:
                        magState = MagazineSubsystem.State.REST;
                        makerState = MakerSubsystem.State.UNITE;
                        break;
                }
            } else {
                switch (mag.getState()) {
                    case REST:
                        magState = MagazineSubsystem.State.DEQUEUE;
                        break;
                }
            }
        }

        //maker manual
        if (fwdMaker && controlLaw.isAtLeast(MIN_MANUAL_TRANSITIONS_LAW)) {
            switch (maker.getState()) {
                case REST:
                    makerState = MakerSubsystem.State.UNITE;
                    break;
                case UNITE:
                    makerState = MakerSubsystem.State.REST;
                    break;
            }
        }
        if (bwdMaker && controlLaw.isAtLeast(MIN_MANUAL_TRANSITIONS_LAW)) {
            switch (maker.getState()) {
                case REST:
                    makerState = MakerSubsystem.State.UNITE;
                    break;
                case UNITE:
                    makerState = MakerSubsystem.State.REST;
                    break;
            }
        }
        //maker auto
        if (maker.jobFulfilled() && controlLaw.isAtLeast(MIN_AUTOMATIC_TRANSITIONS_LAW)) {
            if (makerWasLastForward) {
                switch (maker.getState()) {
                    case UNITE:
                        makerState = MakerSubsystem.State.REST;
                        break;
                }
            } else {
                switch (maker.getState()) {
                    case REST:
                        makerState = MakerSubsystem.State.UNITE;
                        break;
                }
            }
        }

        //clip manual
        if (fwdClip && controlLaw.isAtLeast(MIN_MANUAL_TRANSITIONS_LAW)) {
            switch (clip.getState()) {
                case REST:
                    clipState = ClipSubsystem.State.OPEN;
                    break;
                case OPEN:
                    clipState = ClipSubsystem.State.REST;
                    break;
            }
        }
        if (bwdClip && controlLaw.isAtLeast(MIN_MANUAL_TRANSITIONS_LAW)) {
            switch (clip.getState()) {
                case REST:
                    clipState = ClipSubsystem.State.OPEN;
                    break;
                case OPEN:
                    clipState = ClipSubsystem.State.REST;
                    break;
            }
        }

        if (intakeState == IntakeSubsystem.State.APPROACH_O) {
            intake.powerLift(controls.intakePower);
        }

        intake.setState(intakeState);
        deposit.setState(depositState);
        mag.setState(magState);
        maker.setState(makerState);
        clip.setState(clipState);
    }

    public static double clamp_0p1(double x) {
        return clamp(x, 0, 1);
    }

    public static double clamp_p1m1(double x) {
        return clamp(x, -1, 1);
    }

    public static double clamp(double x, double lower, double upper) {
        return Math.max(Math.min(x, upper), lower);
    }

    public static double sigmoid(double x) {
        return 1/(1+Math.exp(-x));
    }

}
