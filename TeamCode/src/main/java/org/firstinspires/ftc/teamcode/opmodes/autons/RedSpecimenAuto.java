package org.firstinspires.ftc.teamcode.opmodes.autons;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.opmodes.algorithms.SampleDataBufferFilter;
import org.firstinspires.ftc.teamcode.synchropather.AutonomousRobot;
import org.firstinspires.ftc.teamcode.synchropather.macros.ExtendoRetractMacro;
import org.firstinspires.ftc.teamcode.synchropather.subsystemclasses.OverheadCameraSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.Synchronizer;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.TimeSpan;
import org.firstinspires.ftc.teamcode.synchropather.systems.extendo.ExtendoPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.extendo.ExtendoState;
import org.firstinspires.ftc.teamcode.synchropather.systems.extendo.movements.LinearExtendo;
import org.firstinspires.ftc.teamcode.synchropather.systems.hArm.HArmPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.hArm.HArmState;
import org.firstinspires.ftc.teamcode.synchropather.systems.hArm.movements.LinearHArm;
import org.firstinspires.ftc.teamcode.synchropather.systems.hClaw.HClawConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.hClaw.HClawPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.hClaw.movements.GrabHClaw;
import org.firstinspires.ftc.teamcode.synchropather.systems.hClaw.movements.ReleaseHClaw;
import org.firstinspires.ftc.teamcode.synchropather.systems.hWrist.HWristPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.hWrist.movements.MoveHWrist;
import org.firstinspires.ftc.teamcode.synchropather.systems.klipper.KlipperConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.klipper.KlipperPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.klipper.movements.MoveKlipper;
import org.firstinspires.ftc.teamcode.synchropather.systems.lift.LiftConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.lift.LiftPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.lift.LiftState;
import org.firstinspires.ftc.teamcode.synchropather.systems.lift.movements.LinearLift;
import org.firstinspires.ftc.teamcode.synchropather.systems.mFeeder.MFeederConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.mFeeder.MFeederPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.mFeeder.MFeederState;
import org.firstinspires.ftc.teamcode.synchropather.systems.mFeeder.movements.LinearMFeeder;
import org.firstinspires.ftc.teamcode.synchropather.systems.mIntake.MIntakeConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.mIntake.MIntakePlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.mIntake.MIntakeState;
import org.firstinspires.ftc.teamcode.synchropather.systems.mIntake.movements.LinearMIntake;
import org.firstinspires.ftc.teamcode.synchropather.systems.mIntake.movements.MoveMIntake;
import org.firstinspires.ftc.teamcode.synchropather.systems.mLoader.MLoaderConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.mLoader.MLoaderPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.mLoader.movements.MoveMLoader;
import org.firstinspires.ftc.teamcode.synchropather.systems.rotation.RotationConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.rotation.RotationPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.rotation.RotationState;
import org.firstinspires.ftc.teamcode.synchropather.systems.rotation.movements.LinearRotation;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.TranslationConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.TranslationPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.TranslationState;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.movements.CRSplineTranslation;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.movements.DifferentAccelerationsTranslation;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.movements.LinearTranslation;
import org.firstinspires.ftc.teamcode.synchropather.systems.vArm.VArmConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.vArm.VArmPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.vArm.VArmState;
import org.firstinspires.ftc.teamcode.synchropather.systems.vArm.movements.LinearVArm;
import org.firstinspires.ftc.teamcode.synchropather.systems.vClaw.VClawConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.vClaw.VClawPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.vClaw.VClawState;
import org.firstinspires.ftc.teamcode.synchropather.systems.vClaw.movements.GrabVClaw;
import org.firstinspires.ftc.teamcode.synchropather.systems.vClaw.movements.MoveVClaw;
import org.firstinspires.ftc.teamcode.synchropather.systems.vClaw.movements.ReleaseVClaw;

@Config
@Autonomous(name="Red Specimen Auto")
public class RedSpecimenAuto extends LinearOpMode {

    private AutonomousRobot robot;

    private Synchronizer preloadSequence;




    // preload
    public static double liftDownDelay = 0.1;

    // For horizontal intake
    public static double armDownPosition = 1.04;

    public static double spikeMarkIntakeDelay = 0.2;


    // For clipbot subsystem
    private int clipInventory = 0;
    private boolean inventoryStocked = false;

    public static double klipperWaitTime = 0.1;
    public static double feederDelayTime = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        initSubsystems();

        initPreloadSequence();

        waitForStart();

        preloadSequence.start();
        while (opModeIsActive() && preloadSequence.update()) robot.update();
    }


    private void initSubsystems() {
        robot = new AutonomousRobot(
                hardwareMap,
                new Pose2d(24, -72+9, new Rotation2d(Math.toRadians(90))),
                        // back against wall, facing towards sub, centered on first seam from middle
                AutonomousRobot.TeamColor.RED,
                this,
                SampleDataBufferFilter.SampleTargetingMethod.TRANSLATION
        );

        // Horizontal arm slightly up, claw closed
        robot.horizontalIntake.setClawPosition(HClawConstants.GRAB_POSITION);
        robot.horizontalIntake.setWristAngle(0);
        robot.horizontalIntake.setArmPosition(0.9);

        // Vertical arm ready to deposit, vertical claw grabbing
        robot.verticalDeposit.setArmPosition(VArmConstants.armLeftPreDepositPosition);
        robot.verticalDeposit.setClawPosition(VClawConstants.GRAB_POSITION);
    }


    private void initPreloadSequence() {
        TranslationConstants.MAX_VELOCITY = 40d;
        TranslationConstants.MAX_ACCELERATION = 54d;

        RotationConstants.MAX_ANGULAR_VELOCITY = 0.8*3.6;
        RotationConstants.MAX_ANGULAR_ACCELERATION = 0.8*4;

        // Place preloaded specimen
        LinearTranslation scorePreload = new LinearTranslation(0,
                new TranslationState(24, -72+9),
                new TranslationState(1, -24-9+2)
        );
        LinearRotation rotationStill = new LinearRotation(0,
                new RotationState(Math.toRadians(90)),
                new RotationState(Math.toRadians(90))
        );
        LinearLift liftToPreDepositPreload = new LinearLift(scorePreload.getEndTime()-0.25,
                new LiftState(0),
                new LiftState(LiftConstants.preDepositPosition)
        );

        LinearVArm vArmToDepositPreload = new LinearVArm(Math.max(liftToPreDepositPreload.getEndTime(),scorePreload.getEndTime()),
                new VArmState(VArmConstants.armLeftPreDepositPosition),
                new VArmState(VArmConstants.armLeftDepositPosition)
        );

        LinearLift liftToDepositPreload = new LinearLift(vArmToDepositPreload.getEndTime()+0.1,
                new LiftState(LiftConstants.preDepositPosition),
                new LiftState(LiftConstants.depositPosition)
        );

        ReleaseVClaw releaseVClawPreload = new ReleaseVClaw(liftToDepositPreload.getEndTime());

        LinearLift liftDownPreload = new LinearLift(releaseVClawPreload.getEndTime()+ liftDownDelay,
                new LiftState(LiftConstants.depositPosition),
                new LiftState(0)
        );


        // Intake clips from wall
        double previousAcceleration = TranslationConstants.MAX_ACCELERATION;
        TranslationConstants.MAX_ACCELERATION = previousAcceleration/3;
        CRSplineTranslation intakeClipsTranslation = new CRSplineTranslation(liftDownPreload.getEndTime(),
                new TranslationState(1, -24-9+2),
                new TranslationState(5, -48),
                new TranslationState(48.75, -72+9+2)
                        // Y: -72 + 1/2 robot height + mag intake distance from wall
        );
        TranslationConstants.MAX_ACCELERATION = previousAcceleration;


        // Prepare magazine intake and loader
        MoveMIntake intakeOpen = new MoveMIntake(0, MIntakeConstants.openPosition);
        MoveMLoader loaderOpen = new MoveMLoader(0, MLoaderConstants.openPosition);

        // Lift clips
        MoveMIntake intakeUp = new MoveMIntake(
                intakeClipsTranslation.getEndTime(),
                MIntakeConstants.upPosition
        );

        // Move forward toward spike mark samples
        LinearTranslation approachSpikeMark = new LinearTranslation(intakeUp.getEndTime()+0.2,
                new TranslationState(48.75, -72+9+3),
                new TranslationState(48.75, -72+24)
        );

        // Extendo to spike mark sample
        double extension = (-24-3.5/2.0)-(-48) - (OverheadCameraSubsystem.CLAW_OFFSET[0]+OverheadCameraSubsystem.CAMERA_OFFSET[0]) - 1.5;
                           // (spmark) - (bot) - (claw offset)
        LinearExtendo extendToSpikeMark = new LinearExtendo(intakeClipsTranslation.getEndTime()-0.5,
                new ExtendoState(0),
                new ExtendoState(extension)
        );

        // Move horizontal arm down
        LinearHArm h_arm_down = new LinearHArm(Math.max(extendToSpikeMark.getEndTime(),approachSpikeMark.getEndTime())+spikeMarkIntakeDelay/2,
                new HArmState(0.9),
                new HArmState(armDownPosition),
                true
        );
        MoveHWrist h_wrist_align = new MoveHWrist(extendToSpikeMark.getStartTime(), 0);

        // Pick up and move horizontal arm up
        GrabHClaw h_claw_grab = new GrabHClaw(h_arm_down.getEndTime()+spikeMarkIntakeDelay/2, true);
        LinearHArm h_arm_up = new LinearHArm(h_claw_grab.getEndTime(),
                new HArmState(armDownPosition),
                new HArmState(0.9)
        );
        MoveHWrist h_wrist_reset = new MoveHWrist(h_arm_up.getEndTime(), 0, true);

        // Retract extendo
        LinearExtendo extendoIn = new LinearExtendo(h_wrist_reset.getStartTime(),
                new ExtendoState(extension),
                new ExtendoState(3.1)
        );

        // Lower magazine intake
        LinearMIntake intakeClose = new LinearMIntake(
                new TimeSpan(
                        intakeUp.getEndTime() + 1.5,
                        intakeUp.getEndTime() + 3
                ),
                new MIntakeState(MIntakeConstants.upPosition),
                new MIntakeState(MIntakeConstants.closedPosition)
        );

        // Snap samples using loader
        MoveMLoader loaderClose = new MoveMLoader(
                intakeClose.getEndTime(),
                MLoaderConstants.maxClosedPosition
        );
        MoveMLoader loaderRelease = new MoveMLoader(
                loaderClose.getEndTime() + 0.5,
                MLoaderConstants.partialClosedPosition
        );
        MoveMLoader loaderClose2 = new MoveMLoader(
                loaderRelease.getEndTime(),
                MLoaderConstants.maxClosedPosition
        );
        MoveMLoader loaderRelease2 = new MoveMLoader(
                loaderClose2.getEndTime()+0.5,
                MLoaderConstants.openPosition
        );


        // Set clipbot variables
        clipInventory = MFeederConstants.RELOAD_CAPACITY;
        inventoryStocked = true;

        //// Mag has clips, do transfer and clipping sequence
        LinearLift liftUp = new LinearLift(extendoIn.getEndTime(),
                new LiftState(robot.linearSlides.getLeftLiftPosition()),
                new LiftState(LiftConstants.transferPosition)
        );

        // Horizontal arm goes up
        LinearHArm hArmUp = new LinearHArm(Math.max(extendoIn.getEndTime(), liftUp.getEndTime()),
                new HArmState(0.9),
                new HArmState(0.49)
        );

        // Vertical arm gets ready
        LinearVArm vArmDown = new LinearVArm(hArmUp.getEndTime(),
                new VArmState(0.5),
                new VArmState(VArmConstants.armLeftTransferPosition)
        );

        // Deposit claw grabs sample
        GrabVClaw grabVClaw = new GrabVClaw(vArmDown.getEndTime() + 0.25);

        // Intake claw releases sample
        ReleaseHClaw releaseHClaw = new ReleaseHClaw(grabVClaw.getEndTime() + 0.25);

        // Deposit arm moves out of the way
        LinearVArm upVArm = new LinearVArm(releaseHClaw.getEndTime(),
                new VArmState(VArmConstants.armLeftTransferPosition),
                new VArmState(VArmConstants.armLeftTransferPosition-0.1)
        );

        // Intake arm moves back down
        LinearHArm hArmDown = new LinearHArm(upVArm.getEndTime(),
                new HArmState(0.48),
                new HArmState(0.9)
        );

        //// CLIPBOT
        // Get target states
        double maxClips = MFeederConstants.MAX_CAPACITY;
        MFeederState currentFeederPosition = new MFeederState(
                robot.clipbot.getMagazineFeederPosition()
        );

        clipInventory--;
        MFeederState targetFeederPosition = new MFeederState(
                (maxClips - clipInventory) * MFeederConstants.INCHES_PER_CLIP
        );

        /// Movements
        // Hold lift down
        LinearLift holdLiftDown = new LinearLift(hArmDown.getEndTime(),
                new LiftState(LiftConstants.transferPosition),
                new LiftState(-1)
        );

        // Stationary deposit arm
        LinearVArm lowerVArm = new LinearVArm(hArmDown.getEndTime(),
                new VArmState(VArmConstants.armLeftTransferPosition-0.1),
                new VArmState(VArmConstants.armLeftClipperPosition)
        );

        // Loosely hold sample
        MoveVClaw looselyHoldSample = new MoveVClaw(lowerVArm.getEndTime(), 0.1,
                new VClawState(VClawConstants.GRAB_POSITION),
                new VClawState(VClawConstants.LOOSELY_GRABBED_POSITION)
        );

        // Stationary deposit arm
        LinearVArm pressVArm = new LinearVArm(looselyHoldSample.getEndTime(),
                new VArmState(VArmConstants.armLeftClipperPosition+0.02),
                new VArmState(VArmConstants.armLeftClipperPosition+0.02)
        );

        // Advance feeder by one clip
        LinearMFeeder advanceFeeder = new LinearMFeeder(Math.max(loaderRelease2.getEndTime(), looselyHoldSample.getEndTime()) + feederDelayTime,
                currentFeederPosition,
                targetFeederPosition
        );


        // Feeder plan
        MFeederPlan mFeederPlan;
        if (clipInventory==0) {
            LinearMFeeder resetFeeder = new LinearMFeeder(advanceFeeder.getEndTime(),
                    targetFeederPosition,
                    new MFeederState(0)
            );
            mFeederPlan = new MFeederPlan(robot.clipbot,
                    advanceFeeder,
                    resetFeeder
            );
            inventoryStocked = false;
        } else {
            mFeederPlan = new MFeederPlan(robot.clipbot,
                    advanceFeeder
            );
        }

        // Klipper action
        MoveKlipper initKlipper = new MoveKlipper(0, KlipperConstants.openPosition);
        MoveKlipper klipSpecimen = new MoveKlipper(advanceFeeder.getEndTime()+klipperWaitTime, KlipperConstants.closedPosition);
        MoveKlipper unklipSpecimen = new MoveKlipper(klipSpecimen.getEndTime()+0.25, KlipperConstants.openPosition);


        // Score specimen
        CRSplineTranslation splineScoreSpikeMark = new CRSplineTranslation(holdLiftDown.getEndTime()-1.75,
                new TranslationState(48.75, -72+24),
                new TranslationState(7,-46),
                new TranslationState(-1, -24-9+2)
        );

        LinearVArm vArmToPreDepositSpikeMark = new LinearVArm(unklipSpecimen.getEndTime(),
                new VArmState(VArmConstants.armLeftClipperPosition),
                new VArmState(VArmConstants.armLeftPreDepositPosition)
        );

        LinearLift liftToPreDepositSpikeMark = new LinearLift(Math.max(vArmToPreDepositSpikeMark.getStartTime(), splineScoreSpikeMark.getEndTime()),
                new LiftState(0),
                new LiftState(LiftConstants.preDepositPosition)
        );

        LinearVArm vArmToDepositSpikeMark = new LinearVArm(Math.max(liftToPreDepositSpikeMark.getEndTime(), splineScoreSpikeMark.getEndTime()),
                new VArmState(VArmConstants.armLeftPreDepositPosition),
                new VArmState(VArmConstants.armLeftDepositPosition)
        );

        LinearLift liftToDepositSpikeMark = new LinearLift(vArmToDepositSpikeMark.getEndTime()+0.1,
                new LiftState(LiftConstants.preDepositPosition),
                new LiftState(LiftConstants.depositPosition)
        );

        ReleaseVClaw releaseVClawSpikeMark = new ReleaseVClaw(liftToDepositSpikeMark.getEndTime());

        LinearLift liftDownSpikeMark = new LinearLift(releaseVClawSpikeMark.getEndTime()+ liftDownDelay,
                new LiftState(LiftConstants.depositPosition),
                new LiftState(0)
        );






        // Plans
        TranslationPlan translationPlan = new TranslationPlan(robot.drive, robot.localization,
                scorePreload,
                intakeClipsTranslation,
                approachSpikeMark,
                splineScoreSpikeMark
        );
        RotationPlan rotationPlan = new RotationPlan(robot.drive, robot.localization,
                rotationStill
        );
        ExtendoPlan extendoPlan = new ExtendoPlan(robot.linearSlides,
                extendToSpikeMark,
                extendoIn
        );
        HWristPlan h_wrist_plan = new HWristPlan(robot.horizontalIntake,
                h_wrist_align,
                h_wrist_reset
        );
        HArmPlan h_arm_plan = new HArmPlan(robot.horizontalIntake,
                h_arm_down,
                h_arm_up,
                hArmUp,
                hArmDown
        );
        HClawPlan h_claw_plan = new HClawPlan(robot.horizontalIntake,
                h_claw_grab,
                releaseHClaw
        );
        LiftPlan liftPlan = new LiftPlan(robot.linearSlides,
                liftToPreDepositPreload,
                liftToDepositPreload,
                liftDownPreload,
                liftUp,
                holdLiftDown,
                liftToPreDepositSpikeMark,
                liftToDepositSpikeMark,
                liftDownSpikeMark
        );
        VArmPlan vArmPlan = new VArmPlan(robot.verticalDeposit,
                vArmToDepositPreload,
                vArmDown,
                upVArm,
                lowerVArm,
                pressVArm,
                vArmToPreDepositSpikeMark,
                vArmToDepositSpikeMark
        );
        VClawPlan vClawPlan = new VClawPlan(robot.verticalDeposit,
                releaseVClawPreload,
                grabVClaw,
                looselyHoldSample,
                releaseVClawSpikeMark
        );

        KlipperPlan klipperPlan = new KlipperPlan(robot.clipbot,
                initKlipper,
                klipSpecimen,
                unklipSpecimen
        );
        MIntakePlan mIntakePlan = new MIntakePlan(robot.clipbot,
                intakeOpen,
                intakeUp,
                intakeClose
        );
        MLoaderPlan mLoaderPlan = new MLoaderPlan(robot.clipbot,
                loaderOpen,
                loaderClose,
                loaderRelease,
                loaderClose2,
                loaderRelease2
        );


        // Synchronizer
        preloadSequence = new Synchronizer(
                translationPlan,
                rotationPlan,
                extendoPlan,
                h_wrist_plan,
                h_arm_plan,
                h_claw_plan,
                liftPlan,
                vArmPlan,
                vClawPlan,
                klipperPlan,
                mIntakePlan,
                mLoaderPlan,
                mFeederPlan
        );
    }

}
