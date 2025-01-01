package org.firstinspires.ftc.teamcode.subsystems.indep;

import org.firstinspires.ftc.teamcode.HardwareRobot;
import org.firstinspires.ftc.teamcode.RobotSystem;

public class InDepSubsystem {
    // integrates intake, magazine, deposit, and transfer subsystems

    public RobotSystem robot;

    private IntakeSubsystem intake;
    private DepositSubsystem deposit;
    private MagazineSubsystem mag;
    private MakerSubsystem maker;
    private ClipSubsystem clip;

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
    }

    public static class InDepControlData {
        // MANUAL, ALTERNATE
        public boolean intakeButton;
        public boolean depositButton;
        public boolean magButton;
        public boolean makerButton;
        public boolean clipButton;

        // SEMIDIRECT, DIRECT
        public double intakeSlidePower;
        public double leftIntakeSlidePower;
        public double rightIntakeSlidePower;
        public double depositSlidePower;
        public double leftDepositSlidePower;
        public double rightDepositSlidePower;
        public double magPower;
        public double makerPower;
        public double clipPower;

        public void set(
                boolean intakeButton,
                boolean depositButton,
                boolean magButton,
                boolean makerButton,
                boolean clipButton,
                double intakeSlidePower,
                double leftIntakeSlidePower,
                double rightIntakeSlidePower,
                double depositSlidePower,
                double leftDepositSlidePower,
                double rightDepositSlidePower,
                double magPower,
                double makerPower,
                double clipPower
        ) {
            this.intakeButton = intakeButton;
            this.depositButton = depositButton;
            this.magButton = magButton;
            this.makerButton = makerButton;
            this.clipButton = clipButton;
            this.intakeSlidePower = intakeSlidePower;
            this.depositSlidePower = depositSlidePower;
            this.magPower = magPower;
            this.makerPower = makerPower;
            this.clipPower = clipPower;
        }
    }

    private InDepControlData lastControls;

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
        if (controlLaw.isAtMost(ControlLaw.SEMIDIRECT)) tickDirect(controls);
        else tickFSM(controls);

        lastControls.set(
                controls.intakeButton,
                controls.clipButton,
                controls.makerButton,
                controls.magButton,
                controls.depositButton,
                controls.intakeSlidePower,
                controls.depositSlidePower,
                controls.
        );
    }

    public void tickDirect(InDepControlData controls) {

    }

    public void tickFSM(InDepControlData controls) {
        boolean
                fwdIntake = false,
                fwdDeposit = false,
                fwdMag = false,
                fwdMaker = false,
                fwdClip = false;

        if (lastControls.intakeButton && !controls.intakeButton) fwdIntake = true;  // trigger on release
        if (lastControls.depositButton && !controls.depositButton) fwdDeposit = true;
        if (lastControls.magButton && !controls.magButton) fwdMag = true;
        if (lastControls.makerButton && !controls.makerButton) fwdMaker = true;
        if (lastControls.clipButton && !controls.clipButton) fwdClip = true;

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
        // auto-magic pass       sadly, no magic takes place
        if (intake.jobFulfilled() && controlLaw.isAtLeast(MIN_AUTOMATIC_TRANSITIONS_LAW)) {
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
        //deposit auto
        if (deposit.jobFulfilled() && controlLaw.isAtLeast(MIN_AUTOMATIC_TRANSITIONS_LAW)) {
            switch (deposit.getState()) {
                case TRANSFER_O:
                    depositState = DepositSubsystem.State.TRANSFER_C;
                    break;
                case DEPOSIT:
                    depositState = DepositSubsystem.State.REST;
                    break;
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
        //magazine auto
        if (mag.jobFulfilled() && controlLaw.isAtLeast(MIN_AUTOMATIC_TRANSITIONS_LAW)) {
            switch (mag.getState()) {  // for consistent formatting
                case DEQUEUE:
                    magState = MagazineSubsystem.State.REST;
                    makerState = MakerSubsystem.State.UNITE;
                    break;
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
        //maker auto
        if (maker.jobFulfilled() && controlLaw.isAtLeast(MIN_AUTOMATIC_TRANSITIONS_LAW)) {
            switch (maker.getState()) {
                case UNITE:
                    makerState = MakerSubsystem.State.REST;
                    break;
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
