package org.firstinspires.ftc.teamcode.opmodes.autons;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.cvprocessors.SampleOrientationProcessor;
import org.firstinspires.ftc.teamcode.opmodes.algorithms.SampleDataBufferFilter;
import org.firstinspires.ftc.teamcode.synchropather.AutonomousRobot;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.Synchronizer;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.TimeSpan;
import org.firstinspires.ftc.teamcode.synchropather.systems.extendo.ExtendoPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.extendo.ExtendoState;
import org.firstinspires.ftc.teamcode.synchropather.systems.extendo.movements.LinearExtendo;
import org.firstinspires.ftc.teamcode.synchropather.systems.hClaw.HClawConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.klipper.KlipperConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.lift.LiftConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.lift.LiftPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.lift.LiftState;
import org.firstinspires.ftc.teamcode.synchropather.systems.lift.movements.LinearLift;
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
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.movements.LinearTranslation;
import org.firstinspires.ftc.teamcode.synchropather.systems.vArm.VArmConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.vArm.VArmPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.vArm.VArmState;
import org.firstinspires.ftc.teamcode.synchropather.systems.vArm.movements.LinearVArm;
import org.firstinspires.ftc.teamcode.synchropather.systems.vClaw.VClawConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.vClaw.VClawPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.vClaw.movements.ReleaseVClaw;

@Config
@Autonomous(name="Specimen Auto")
public class SpecimenAuto extends LinearOpMode {

    private AutonomousRobot robot;

    private Synchronizer preloadSequence;




    // preload
    public static double liftDownDelay = 0.1;
    public static double loaderFeedingPosition = 0.075;

    @Override
    public void runOpMode() throws InterruptedException {
        initSubsystems();

        initPreloadSequence();

        waitForStart();


        /// Score preload and one spike mark
        preloadSequence.start();
        while (opModeIsActive() && preloadSequence.update()) {
            robot.update();
        }
        preloadSequence.stop();

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

        // Klipper up
        robot.clipbot.setKlipperPosition(KlipperConstants.openPosition);

        // init sample orientation processor
        if (robot.teamColor.equals(AutonomousRobot.TeamColor.BLUE)) {
            SampleOrientationProcessor.colorType = SampleOrientationProcessor.SampleColor.BLUE;
        } else {
            SampleOrientationProcessor.colorType = SampleOrientationProcessor.SampleColor.RED;
        }
    }


    private void initPreloadSequence() {
        TranslationConstants.MAX_VELOCITY = 40d;
        TranslationConstants.MAX_ACCELERATION = 54d;

        RotationConstants.MAX_ANGULAR_VELOCITY = 0.8*3.6;
        RotationConstants.MAX_ANGULAR_ACCELERATION = 0.8*4;

        // Place preloaded specimen
        double previousAcceleration = TranslationConstants.MAX_ACCELERATION;
        TranslationConstants.MAX_ACCELERATION = previousAcceleration/3;
        LinearTranslation scorePreload = new LinearTranslation(0,
                new TranslationState(24, -72+9),
                new TranslationState(1, -24-9+2)
        );
        LinearRotation rotationStill = new LinearRotation(0,
                new RotationState(Math.toRadians(90)),
                new RotationState(Math.toRadians(90))
        );
        LinearExtendo extendoStill = new LinearExtendo(0,
                new ExtendoState(0),
                new ExtendoState(0)
        );
        LinearLift liftToPreDepositPreload = new LinearLift(scorePreload.getEndTime(),
                new LiftState(0),
                new LiftState(LiftConstants.preDepositPosition)
        );

        LinearVArm vArmToDepositPreload = new LinearVArm(Math.max(liftToPreDepositPreload.getEndTime(),scorePreload.getEndTime()),
                new VArmState(VArmConstants.armLeftPreDepositPosition),
                new VArmState(VArmConstants.armLeftDepositPosition)
        );

        LinearLift liftToDepositPreload = new LinearLift(vArmToDepositPreload.getEndTime()+0.25,
                new LiftState(LiftConstants.preDepositPosition),
                new LiftState(LiftConstants.depositPosition)
        );

        ReleaseVClaw releaseVClawPreload = new ReleaseVClaw(liftToDepositPreload.getEndTime());

        LinearLift liftDownPreload = new LinearLift(releaseVClawPreload.getEndTime()+ liftDownDelay,
                new LiftState(LiftConstants.depositPosition),
                new LiftState(0)
        );



        // Intake clips from wall
        CRSplineTranslation intakeClipsTranslation = new CRSplineTranslation(liftDownPreload.getEndTime(),
                new TranslationState(1, -24-9+2),
                new TranslationState(5, -48),
                new TranslationState(47.75, -72+9+4)
                        // Y: -72 + 1/2 robot height + mag intake distance from wall
        );


        // Prepare magazine intake and loader
        MoveMIntake intakeOpen = new MoveMIntake(0, MIntakeConstants.openPosition);
        MoveMLoader loaderOpen = new MoveMLoader(0, MLoaderConstants.openPosition);


        LinearTranslation alignClipsVertically = new LinearTranslation(intakeClipsTranslation.getEndTime()+5,
                new TranslationState(48.75, -72+9+4),
                new TranslationState(48.75, -72+9+2)
        );

        // Partially lift intake
        MoveMIntake intakePartiallyUp = new MoveMIntake(
                alignClipsVertically.getEndTime()+1,
                MIntakeConstants.partiallyUpPosition
        );

        // Lift clips
        MoveMIntake intakeUp = new MoveMIntake(
                intakePartiallyUp.getEndTime()+1,
                MIntakeConstants.upPosition
        );

        // Move forward toward spike mark samples
        LinearTranslation approachSpikeMark = new LinearTranslation(intakeUp.getEndTime()+0.2,
                new TranslationState(48.75, -72+9+2),
                new TranslationState(48.75, -72+18)
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
                loaderClose.getEndTime() + 1,
                MLoaderConstants.partialClosedPosition
        );
        MoveMLoader loaderClose2 = new MoveMLoader(
                loaderRelease.getEndTime(),
                MLoaderConstants.maxClosedPosition
        );
        MoveMLoader loaderRelease2 = new MoveMLoader(
                loaderClose2.getEndTime()+0.5,
                MLoaderConstants.partialClosedPosition
        );
        MoveMLoader loaderClose3 = new MoveMLoader(
                loaderRelease2.getEndTime(),
                MLoaderConstants.maxClosedPosition
        );
        MoveMLoader loaderRelease3 = new MoveMLoader(
                loaderClose3.getEndTime()+0.5,
                loaderFeedingPosition
        );



        TranslationConstants.MAX_ACCELERATION = previousAcceleration;



        TranslationPlan translationPlan = new TranslationPlan(robot.drive, robot.localization,
                scorePreload,
                intakeClipsTranslation,
                alignClipsVertically,
                approachSpikeMark
        );
        RotationPlan rotationPlan = new RotationPlan(robot.drive, robot.localization,
                rotationStill
        );
        ExtendoPlan extendoPlan = new ExtendoPlan(robot.linearSlides,
                extendoStill
        );
        LiftPlan liftPlan = new LiftPlan(robot.linearSlides,
                liftToPreDepositPreload,
                liftToDepositPreload,
                liftDownPreload
        );
        VArmPlan vArmPlan = new VArmPlan(robot.verticalDeposit,
                vArmToDepositPreload
        );
        VClawPlan vClawPlan = new VClawPlan(robot.verticalDeposit,
                releaseVClawPreload
        );
        MIntakePlan mIntakePlan = new MIntakePlan(robot.clipbot,
                intakeOpen,
                intakePartiallyUp,
                intakeUp,
                intakeClose
        );
        MLoaderPlan mLoaderPlan = new MLoaderPlan(robot.clipbot,
                loaderOpen,
                loaderClose,
                loaderRelease,
                loaderClose2,
                loaderRelease2,
                loaderClose3,
                loaderRelease3
        );


        // Synchronizer
        preloadSequence = new Synchronizer(
                translationPlan,
                rotationPlan,
                extendoPlan,
                liftPlan,
                vArmPlan,
                vClawPlan,
                mIntakePlan,
                mLoaderPlan
        );
    }






    /**
     * Normalizes a given angle to (-pi,pi] radians.
     * @param radians the given angle in radians.
     * @return the normalized angle in radians.
     */
    private static double normalizeAngle(double radians) {
        while (radians >= Math.PI) radians -= 2*Math.PI;
        while (radians < -Math.PI) radians += 2*Math.PI;
        return radians;
    }

}
