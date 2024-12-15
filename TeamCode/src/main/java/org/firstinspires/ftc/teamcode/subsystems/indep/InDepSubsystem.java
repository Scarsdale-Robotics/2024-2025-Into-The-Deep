package org.firstinspires.ftc.teamcode.subsystems.indep;

import org.firstinspires.ftc.teamcode.HardwareRobot;
import org.firstinspires.ftc.teamcode.RobotSystem;

public class InDepSubsystem {
    // integrates intake, magazine, deposit, and transfer subsystems

    public String[] movements;

    public RobotSystem robot;

    private IntakeSubsystem intake;
    private DepositSubsystem deposit;
    private MagazineSubsystem mag;
    private MakerSubsystem maker;
    private ClipSubsystem clip;

    public InDepSubsystem(RobotSystem robot, HardwareRobot hardware) {
        this.robot = robot;

        this.intake = new IntakeSubsystem(hardware);
        this.deposit = new DepositSubsystem(hardware);
        this.mag = new MagazineSubsystem(hardware);
        this.maker = new MakerSubsystem(hardware);
        this.clip = new ClipSubsystem(hardware);
    }

    private boolean
            lastFwdIntake = false,
            lastFwdDeposit = false,
            lastFwdMag = false,
            lastFwdMaker = false,
            lastFwdClip = false;

    private IntakeSubsystem.State intakeState;
    private DepositSubsystem.State depositState;
    private MagazineSubsystem.State magState;
    private MakerSubsystem.State makerState;
    private ClipSubsystem.State clipState;

    public void tick(
            boolean intakeButton,
            boolean depositButton,
            boolean magButton,
            boolean makerButton,
            boolean clipButton
    ) {
        boolean
                fwdIntake = false,
                fwdDeposit = false,
                fwdMag = false,
                fwdMaker = false,
                fwdClip = false;

        if (lastFwdIntake && !intakeButton) fwdIntake = true;  // trigger on release
        if (lastFwdDeposit && !depositButton) fwdDeposit = true;
        if (lastFwdMag && !magButton) fwdMag = true;
        if (lastFwdMaker && !makerButton) fwdMaker = true;
        if (lastFwdClip && !clipButton) fwdMaker = true;

        // manual pass
        if (fwdIntake) {
            switch (intake.getState()) {
                case REST:
                    intakeState = IntakeSubsystem.State.APPROACH_O;
                    break;
                case INTAKE_O:
                    intakeState = IntakeSubsystem.State.INTAKE_O;
                    break;
                case INTAKE_C:
                    intakeState = IntakeSubsystem.State.INTAKE_C;
                    break;
                case APPROACH_C:
                    intakeState = IntakeSubsystem.State.TRANSFER_C;
                    magState = MagazineSubsystem.State.DEQUEUE;
                    break;
                case TRANSFER_C:
                    intakeState = IntakeSubsystem.State.TRANSFER_O;
                    break;
                case TRANSFER_O:
                    intakeState = IntakeSubsystem.State.REST;
                    makerState = MakerSubsystem.State.UNITE;
                    break;
            }
        }
        // auto-magic pass
        if (intake.jobFulfilled()) {
            switch (intake.getState()) {
                case INTAKE_O:
                    intakeState = IntakeSubsystem.State.INTAKE_C;
                    break;
                case APPROACH_C:
                    magState = MagazineSubsystem.State.DEQUEUE;
                    break;
                case TRANSFER_C:
                    intakeState = IntakeSubsystem.State.TRANSFER_C;
                    break;
                case TRANSFER_O:
                    makerState = MakerSubsystem.State.UNITE;
                    break;
            }
        }

        if (fwdDeposit) {
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
        if (deposit.jobFulfilled()) {
            switch (deposit.getState()) {
                case TRANSFER_O:
                    depositState = DepositSubsystem.State.TRANSFER_C;
                    break;
                case DEPOSIT:
                    depositState = DepositSubsystem.State.REST;
                    break;
            }
        }

        if (fwdMag) {
            switch (mag.getState()) {
                case REST:
                    magState = MagazineSubsystem.State.DEQUEUE;
                    break;
                case DEQUEUE:
                    magState = MagazineSubsystem.State.REST;
                    break;
            }
        }
        if (mag.jobFulfilled()) {
            switch (mag.getState()) {  // for consistent formatting
                case DEQUEUE:
                    magState = MagazineSubsystem.State.REST;
                    break;
            }
        }

        if (fwdMaker) {
            switch (maker.getState()) {
                case REST:
                    makerState = MakerSubsystem.State.UNITE;
                    break;
                case UNITE:
                    makerState = MakerSubsystem.State.REST;
                    depositState = DepositSubsystem.State.TRANSFER_O;
                    break;
            }
        }
        if (maker.jobFulfilled()) {
            switch (maker.getState()) {
                case UNITE:
                    makerState = MakerSubsystem.State.REST;
                    depositState = DepositSubsystem.State.TRANSFER_O;
                    break;
            }
        }

        intake.setState(intakeState);
        deposit.setState(depositState);
        mag.setState(magState);
        maker.setState(makerState);
        clip.setState(clipState);
    }

}
