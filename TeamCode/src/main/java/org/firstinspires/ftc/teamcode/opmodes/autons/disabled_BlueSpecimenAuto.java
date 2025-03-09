package org.firstinspires.ftc.teamcode.opmodes.autons;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.cvprocessors.SampleOrientationProcessor;
import org.firstinspires.ftc.teamcode.opmodes.algorithms.SampleDataBufferFilter;
import org.firstinspires.ftc.teamcode.synchropather.AutonomousRobot;
import org.firstinspires.ftc.teamcode.synchropather.macros.EducatedSearchMacro;
import org.firstinspires.ftc.teamcode.synchropather.macros.ExtendoRetractMacro;
import org.firstinspires.ftc.teamcode.synchropather.macros.ShimmyMacro;
import org.firstinspires.ftc.teamcode.synchropather.subsystemclasses.OverheadCameraSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.Synchronizer;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.TimeSpan;
import org.firstinspires.ftc.teamcode.synchropather.systems.extendo.ExtendoConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.extendo.ExtendoPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.extendo.ExtendoState;
import org.firstinspires.ftc.teamcode.synchropather.systems.extendo.movements.DynamicLinearExtendo;
import org.firstinspires.ftc.teamcode.synchropather.systems.extendo.movements.LinearExtendo;
import org.firstinspires.ftc.teamcode.synchropather.systems.hArm.HArmConstants;
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
import org.firstinspires.ftc.teamcode.synchropather.systems.limelight.LimelightPipeline;
import org.firstinspires.ftc.teamcode.synchropather.systems.limelight.LimelightPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.limelight.movements.DisableLimelight;
import org.firstinspires.ftc.teamcode.synchropather.systems.limelight.movements.EnableLimelight;
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
import org.firstinspires.ftc.teamcode.synchropather.systems.mLoader.MLoaderState;
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
import org.firstinspires.ftc.teamcode.synchropather.systems.vClaw.VClawState;
import org.firstinspires.ftc.teamcode.synchropather.systems.vClaw.movements.MoveVClaw;
import org.firstinspires.ftc.teamcode.synchropather.systems.vClaw.movements.ReleaseVClaw;

import java.io.FileWriter;
import java.io.IOException;
import java.util.List;

@Config
//@Disabled
@Autonomous(name="[DISABLED] Blue Specimen Auto")
public class disabled_BlueSpecimenAuto extends LinearOpMode {

    private AutonomousRobot robot;

    private Synchronizer preloadSequence;
    private Synchronizer retractMacro;
    private Synchronizer shimmyMacro;
    private Synchronizer searchMacro;
    private Synchronizer pickupMacro;
    private Synchronizer makerMacro;
    private Synchronizer depositMacro;

    private Synchronizer enableLimelightAction;
    private Synchronizer disableLimelightAction;




    // preload
    public static double liftDownDelay = 0.1;

    public static double armDownPosition = 1.025;

    public static double spikeMarkIntakeDelay = 0.2;


    // For clipbot subsystem
    private int clipInventory = 0;
    private boolean inventoryStocked = false;
    public static double klipperWaitTime = 0.3;
    public static double feederDelayTime = -0.09;
    public static double holdLiftDownPosition = -1;


    // preload to cycling
    private double timeToExtendAfterDeposit;


    // cycling
    public static double y_camera = 0;//-0.57047244;

    public static double driveSpeed = 1;
    public static int overheadFilterLength = 7;
    public static double intakeDelay = 0.25;
    public static double minSampleX = -2;
    public static double maxSampleX = 4;

    public static double advanceTime = 1.1;


    // Deposit
    public static double liftUpDepositDelay = 0.25;



    // Checking if claw picked up anything
    private double pickupCheckClawTime;
    public static double clawCheckPosition = 0.79;
    public static double checkPixelsDecayFactor = 1;
    public static double pickupCheckClawTimeDelay = 0.25;


    // loader applying pressure on clips
    public static double loaderPressurePosition = 0.02;
    public static double loaderFeedingPosition = 0.075;



    // Optimization
    public static double horizontalArmRaiseDeadtime = 0.43;
    public static double verticalLiftDownDeadtime = 0.23;


    // Auto -> teleop transition
    public static String filePath = "../magazine_position.txt";


    @Override
    public void runOpMode() throws InterruptedException {
        initSubsystems();
//
//        initPreloadSequence();

        waitForStart();


        /// Score preload and one spike mark
        preloadSequence.start();
        boolean foundSampleToPickup = false;
        while (opModeIsActive()) {
            updateRobot();
            boolean keepRunning;
            keepRunning = preloadSequence.update();
//            // Check to pre extend for sample intake
//            double currentElapsedTime = preloadSequence.getElapsedTime();
//            if (currentElapsedTime < timeToExtendAfterDeposit) {
//                keepRunning = preloadSequence.update();
//            } else {
//                keepRunning = preloadSequence.updateExcluding(
//                        MovementType.TRANSLATION,
//                        MovementType.ROTATION,
//                        MovementType.EXTENDO,
//                        MovementType.HORIZONTAL_ARM,
//                        MovementType.HORIZONTAL_WRIST,
//                        MovementType.HORIZONTAL_CLAW
//                );
//                // init search macro
//                if (searchMacro==null) {
//                    double[] foundSample = getClosestLLSample();
//                    if (foundSample!=null) {
//                        // init search
//                        searchMacro = new EducatedSearchMacro(
//                                foundSample,
//                                robot,
//                                1
//                        );
//                    }
//                    searchMacro.start();
//                    robot.overheadSampleData.clearFilterData();
//                    foundSampleToPickup = true;
//                }
//                if (searchMacro!=null) {
//                    // extend loop until sample has been found
//                    boolean dataFilterFull = robot.overheadSampleData.isFilterFull();
//                    keepRunning = keepRunning || !dataFilterFull;
//                    // extend loop until end of macro
//                    boolean searchMacroRunning = searchMacro.update();
//                    if (!searchMacroRunning) {
//                        keepRunning = false;
//                    }
//                }
//            }
            // break condition
            if (!keepRunning) {
                break;
            }
        }
        preloadSequence.stop();


        /// Cycling
        boolean leftFirst = true;
        while (opModeIsActive()) {

            // TODO: park code? this is never going to happen probably
            if (!inventoryStocked) break;

            // intake
            while (opModeIsActive()) {
                // look for samples
                retractMacro = new ExtendoRetractMacro(
                        robot.linearSlides,
                        robot.horizontalIntake
                );
                retractMacro.start();
                while (opModeIsActive() && retractMacro.update()) updateRobot();
                retractMacro.stop();

                // shimmy in front of sub and look for samples
                enableLimelightAction.update();
                double[] foundSample = null;
                while (opModeIsActive()) {
                    shimmyMacro = new ShimmyMacro(robot.drive, robot.localization, leftFirst);
                    shimmyMacro.start();
                    while (opModeIsActive() && shimmyMacro.update()) {
                        updateRobot();
                        foundSample = getClosestLLSample();
                        if (foundSample != null) break;
                    }
                    shimmyMacro.stop();
                    if (foundSample != null) break;
                }
                disableLimelightAction.update();

                // extend to sample
                searchMacro = new EducatedSearchMacro(
                        foundSample,
                        robot,
                        1
                );
                searchMacro.start();
                robot.overheadSampleData.clearFilterData();
                while (opModeIsActive() && !robot.overheadSampleData.isFilterFull()) {
                    updateRobot();
                    searchMacro.update();
                }

                // try to pick it up
                initPickupMacro(new ExtendoState(0));
                pickupMacro.start();
                while (opModeIsActive() && pickupMacro.update()) updateRobot();
                pickupMacro.stop();

                // check if claw has sample
                boolean hasSample = !robot.clawVacancyProcessor.isClawEmpty();
                if (hasSample) break;
                else {
                    leftFirst = !leftFirst;
                }
            }


            // maker
            initMakerMacro();
            makerMacro.start();
            while (opModeIsActive() && makerMacro.update()) updateRobot();
            clipInventory--;
            if (clipInventory==0) {
                inventoryStocked = false;
            }
            robot.clipbot.setMagazineFeederPower(0);
            robot.linearSlides.stopLifts();
            robot.linearSlides.stopExtendo();


            // TODO: Consider standard end pos to prevent spec attach conflict


            // move right
            LinearTranslation linearTranslation = new LinearTranslation(0,
                    new TranslationState(robot.localization.getPose()),
                    new TranslationState(5,-24-9+2)
            );
            TranslationPlan translationPlan = new TranslationPlan(robot.drive, robot.localization, linearTranslation);
            LinearRotation rotationStill = new LinearRotation(0, new RotationState(Math.PI/2), new RotationState(Math.PI/2));
            RotationPlan rotationPlan = new RotationPlan(robot.drive, robot.localization, rotationStill);
            shimmyMacro = new Synchronizer(translationPlan, rotationPlan);
            shimmyMacro.start();
            while (opModeIsActive() && shimmyMacro.update()) updateRobot();
            shimmyMacro.stop();


            // deposit
            initDepositMacro();
            depositMacro.start();
            while (opModeIsActive() && depositMacro.update()) updateRobot();
            robot.linearSlides.stopLifts();
            robot.verticalDeposit.release();
            leftFirst = !leftFirst;


            if (true) break;

        }


    }

    private double[] getClosestLLSample() {
        Pose2d botPose = robot.localization.getPose();
        List<double[]> samplePositions;
        if (robot.teamColor==AutonomousRobot.TeamColor.BLUE) {
            samplePositions = robot.limelightSubsystem.getBlueSamplePositions();
        } else {
            samplePositions = robot.limelightSubsystem.getRedSamplePositions();
        }
        double[] foundSample = null;
        if (!samplePositions.isEmpty()) {
            double closestDistance = Double.MAX_VALUE;
            double[] closestSample = null;
            for (double[] samplePosition : samplePositions) {
                // Check if reachable
                double targetExtension = getTargetExtension(samplePosition);
                if (samplePosition[0] < minSampleX || maxSampleX < samplePosition[0] || targetExtension > ExtendoConstants.MAX_EXTENSION-SampleDataBufferFilter.sampleRejectionExtendoDistanceBias) continue;
                // compare against best
                double distance = Math.hypot(samplePosition[0]-botPose.getX(), samplePosition[1]-botPose.getY());
                if (distance < closestDistance) {
                    closestDistance = distance;
                    closestSample = samplePosition;
                }
            }
            foundSample = closestSample;
        }
        return foundSample;
    }

    private double getTargetExtension(double[] samplePosition) {
        // Unpack bot pose
        Pose2d botPose = robot.localization.getPose();
        double x_bot = botPose.getX();
        double y_bot = botPose.getY();
        double heading_bot = botPose.getHeading();

        // Unpack sample pose
        double x_sample = samplePosition[0];
        double y_sample = samplePosition[1];

        // Get extendo state
        double x_extendo_min = OverheadCameraSubsystem.CAMERA_OFFSET[0] + OverheadCameraSubsystem.CLAW_OFFSET[0];

        // Calculate target positions
        double sin = Math.sin(heading_bot);
        double cos = Math.cos(heading_bot);
        double T = (x_sample-x_bot)*sin - (y_sample-y_bot)*cos;
        double x_target_center = x_bot + T*sin;
        double y_target_center = y_bot - T*cos;

        ExtendoState extendoTarget = new ExtendoState(
                Math.max(0, Math.hypot(x_sample-x_target_center, y_sample-y_target_center) - x_extendo_min)
        );

        return extendoTarget.getLength();
    }


    private void initSubsystems() {
        robot = new AutonomousRobot(
                hardwareMap,
                new Pose2d(24, -72+9, new Rotation2d(Math.toRadians(90))),
                        // back against wall, facing towards sub, centered on first seam from middle
                AutonomousRobot.TeamColor.BLUE,
                this,
                SampleDataBufferFilter.SampleTargetingMethod.TRANSLATION
        );
//        robot.overheadSampleData.setFilterLength(overheadFilterLength);

//        // Horizontal arm slightly up, claw closed
//        robot.horizontalIntake.setClawPosition(HClawConstants.GRAB_POSITION);
//        robot.horizontalIntake.setWristAngle(0);
//        robot.horizontalIntake.setArmPosition(0.9);
//
//        // Vertical arm ready to deposit, vertical claw grabbing
//        robot.verticalDeposit.setArmPosition(VArmConstants.armLeftPreDepositPosition);
//        robot.verticalDeposit.setClawPosition(VClawConstants.GRAB_POSITION);
//
//        // Klipper up
//        robot.clipbot.setKlipperPosition(KlipperConstants.openPosition);
//
//        // init limelight actions
//        EnableLimelight enableLimelight = new EnableLimelight(new TimeSpan(0,10), LimelightPipeline.SAMPLE_DETECTOR);
//        LimelightPlan enableLimelightPlan = new LimelightPlan(robot.limelightSubsystem, enableLimelight);
//        this.enableLimelightAction = new Synchronizer(enableLimelightPlan);
//        this.enableLimelightAction.start();
//
//        DisableLimelight disableLimelight = new DisableLimelight(new TimeSpan(0,10));
//        LimelightPlan disableLimelightPlan = new LimelightPlan(robot.limelightSubsystem, disableLimelight);
//        this.disableLimelightAction = new Synchronizer(disableLimelightPlan);
//        this.disableLimelightAction.start();
//
//        // init sample orientation processor
//        if (robot.teamColor.equals(AutonomousRobot.TeamColor.BLUE)) {
//            SampleOrientationProcessor.colorType = SampleOrientationProcessor.SampleColor.BLUE;
//        } else {
//            SampleOrientationProcessor.colorType = SampleOrientationProcessor.SampleColor.RED;
//        }
    }

    private void updateRobot() {
        robot.update();
        robot.overheadSampleData.updateFilterData(robot.overheadCamera.getSamplePositions(), robot.overheadCamera.getSampleAngles(), robot.overheadCamera.getClosestSample()); // Can return null

        // write mag position to text file
        try (FileWriter writer = new FileWriter(filePath, false)) { // 'false' ensures overwriting
            writer.write((int)MFeederConstants.inchesToTicks(robot.clipbot.getMagazineFeederPosition()+MFeederConstants.ZERO_HOME));
            writer.write(System.lineSeparator());
        } catch (IOException e) {
            telemetry.addData("FILE ERROR! FILE ERROR! FILE ERROR! ","");
            telemetry.update();
        }
    }

    public static double thing = 0.25;  // delay between klip and raise

    private void initPreloadSequence() {
        TranslationConstants.MAX_VELOCITY = 60d;
        TranslationConstants.MAX_ACCELERATION = 54d;

        RotationConstants.MAX_ANGULAR_VELOCITY = 0.8*3.6;
        RotationConstants.MAX_ANGULAR_ACCELERATION = 0.8*4;

        // Place preloaded specimen
        EnableLimelight enableLimelight = new EnableLimelight(new TimeSpan(0,1), LimelightPipeline.SAMPLE_DETECTOR);
        LinearTranslation scorePreload = new LinearTranslation(0,
                new TranslationState(24, -72+9),
                new TranslationState(1, -24-9+2)
        );
        LinearRotation rotationStill = new LinearRotation(0,
                new RotationState(Math.toRadians(90)),
                new RotationState(Math.toRadians(90))
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

        DisableLimelight disableLimelight = new DisableLimelight(new TimeSpan(liftDownPreload.getEndTime(),liftDownPreload.getEndTime()+1));


        // Intake clips from wall
        double previousAcceleration = TranslationConstants.MAX_ACCELERATION;
        TranslationConstants.MAX_ACCELERATION = previousAcceleration/3;
        LinearTranslation intakeClipsTranslation = new LinearTranslation(releaseVClawPreload.getEndTime(),
                new TranslationState(1, -24-9+2),
                new TranslationState(47.75, -72+9+2)
                        // Y: -72 + 1/2 robot height + mag intake distance from wall
        );
        TranslationConstants.MAX_ACCELERATION = previousAcceleration;


        // Prepare magazine intake and loader
        MoveMIntake intakeOpen = new MoveMIntake(0, MIntakeConstants.openPosition);
        MoveMLoader loaderOpen = new MoveMLoader(0, MLoaderConstants.openPosition);


        // Partially lift intake
        MoveMIntake intakePartiallyUp = new MoveMIntake(
                intakeClipsTranslation.getEndTime(),
                MIntakeConstants.partiallyUpPosition
        );

        LinearTranslation alignClips = new LinearTranslation(intakePartiallyUp.getEndTime(),
                new TranslationState(47.75, -72+9+2), //TODO: changed y to -72+9+2.25
                new TranslationState(48.75, -72+9+2)
        );

        // Lift clips
        MoveMIntake intakeUp = new MoveMIntake(
                alignClips.getEndTime(),
                MIntakeConstants.upPosition
        );

        // Move forward toward spike mark samples
        LinearTranslation approachSpikeMark = new LinearTranslation(intakeUp.getEndTime()+0.2,
                new TranslationState(48.75, -72+9+2),
                new TranslationState(48.75, -72+24)
        );

        // Extendo to spike mark sample
        double extension = (-24-3.5/2.0)-(-48) - (OverheadCameraSubsystem.CLAW_OFFSET[0]+OverheadCameraSubsystem.CAMERA_OFFSET[0]) - 1.5;
                           // (spmark) - (bot) - (claw offset)
        LinearExtendo extendToSpikeMark = new LinearExtendo(intakeClipsTranslation.getEndTime()-0.5,
                new ExtendoState(0),
                new ExtendoState(extension)
        );

        // Lift gets ready for transfer
        LinearLift liftUp = new LinearLift(extendToSpikeMark.getStartTime(),
                new LiftState(robot.linearSlides.getLeftLiftPosition()),
                new LiftState(LiftConstants.transferPosition)
        );

        // Move horizontal arm down
        LinearHArm h_arm_down = new LinearHArm(Math.max(extendToSpikeMark.getEndTime(),approachSpikeMark.getEndTime())+spikeMarkIntakeDelay/2,
                new HArmState(0.9),
                new HArmState(armDownPosition),
                true
        );
        MoveHWrist h_wrist_align = new MoveHWrist(extendToSpikeMark.getStartTime(), 0);

        // Pick up and move horizontal arm up
        GrabHClaw h_claw_grab = new GrabHClaw(h_arm_down.getEndTime()+spikeMarkIntakeDelay, true);
        LinearHArm h_arm_up = new LinearHArm(h_claw_grab.getEndTime()+spikeMarkIntakeDelay,
                new HArmState(armDownPosition),
                new HArmState(HArmConstants.armTransferPosition)
        );
        MoveHWrist h_wrist_reset = new MoveHWrist(h_arm_up.getEndTime(), 0, true);

        // Retract extendo
        double previousExtendoAcceleration = ExtendoConstants.MAX_ACCELERATION;
        ExtendoConstants.MAX_ACCELERATION = 30;
        LinearExtendo extendoIn = new LinearExtendo(h_wrist_reset.getStartTime(),
                new ExtendoState(extension),
                new ExtendoState(ExtendoConstants.transferPosition+0.5)
        );
        ExtendoConstants.MAX_ACCELERATION = previousExtendoAcceleration;

        // Lower magazine intake
        LinearMIntake intakeClose = new LinearMIntake(
                new TimeSpan(
                        intakeUp.getEndTime() + 1.5,
                        intakeUp.getEndTime() + 5
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


        // Set clipbot variables
        clipInventory = 6;
        inventoryStocked = true;

        //// Mag has clips, do transfer and clipping sequence
        // Vertical arm gets ready
        LinearVArm vArmDown = new LinearVArm(Math.max(h_arm_up.getEndTime(), extendoIn.getEndTime()),
                new VArmState(VArmConstants.armLeftDepositPosition),
                new VArmState(VArmConstants.armLeftTransferPosition)
        );

        // Loosely hold sample
        MoveVClaw looselyHoldSampleTransfer = new MoveVClaw(vArmDown.getEndTime()+0.25, 0.1,
                new VClawState(VClawConstants.RELEASE_POSITION),
                new VClawState(VClawConstants.LOOSELY_GRABBED_POSITION)
        );

        // Pull sample in more
        LinearExtendo pullSampleIn = new LinearExtendo(looselyHoldSampleTransfer.getEndTime(),
                new ExtendoState(ExtendoConstants.transferPosition+0.5),
                new ExtendoState(0)
        );

        // Deposit claw grabs sample
        MoveVClaw grabVClaw = new MoveVClaw(pullSampleIn.getEndTime(), 0.1,
                new VClawState(VClawConstants.LOOSELY_GRABBED_POSITION),
                new VClawState(VClawConstants.GRAB_POSITION)
        );

        // Intake claw releases sample
        ReleaseHClaw releaseHClaw = new ReleaseHClaw(grabVClaw.getStartTime());

        // Deposit arm moves out of the way
        LinearVArm upVArm = new LinearVArm(releaseHClaw.getEndTime(),
                new VArmState(VArmConstants.armLeftTransferPosition),
                new VArmState(VArmConstants.armLeftTransferPosition-0.1)
        );

        // Intake arm moves back down
        LinearHArm hArmDown = new LinearHArm(upVArm.getEndTime(),
                new HArmState(HArmConstants.armTransferPosition),
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
        LinearVArm lowerVArm = new LinearVArm(holdLiftDown.getEndTime()-0.25,
                new VArmState(VArmConstants.armLeftTransferPosition-0.1),
                new VArmState(VArmConstants.armLeftClipperPosition)
        );

        // Stationary deposit arm
        LinearVArm pressVArm = new LinearVArm(holdLiftDown.getEndTime(),
                new VArmState(VArmConstants.armLeftClipperPosition+0.02),
                new VArmState(VArmConstants.armLeftClipperPosition+0.02)
        );

        // Advance feeder by one clip
        double ti = Math.max(loaderRelease3.getEndTime(), holdLiftDown.getEndTime()) + feederDelayTime;
        LinearMFeeder advanceFeeder = new LinearMFeeder(new TimeSpan(ti, ti+advanceTime),
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

        MoveMLoader loaderApplyPressure = new MoveMLoader(advanceFeeder.getEndTime()+0.25, loaderPressurePosition);

        // Klipper action
        MoveKlipper initKlipper = new MoveKlipper(0, KlipperConstants.openPosition);
        MoveKlipper klipSpecimen = new MoveKlipper(advanceFeeder.getEndTime()+klipperWaitTime, KlipperConstants.closedPosition);
        MoveKlipper unklipSpecimen = new MoveKlipper(klipSpecimen.getEndTime(), 0.7);

        // Score specimen
        TranslationConstants.MAX_ACCELERATION = previousAcceleration/3;
        CRSplineTranslation splineScoreSpikeMark = new CRSplineTranslation(holdLiftDown.getEndTime()-1.75,
                new TranslationState(48.75, -72+24),
                new TranslationState(7,-46),
                new TranslationState(-2, -24-9+2)
        );
        TranslationConstants.MAX_ACCELERATION = previousAcceleration;

        LinearVArm vArmToPreDepositSpikeMark = new LinearVArm(unklipSpecimen.getEndTime()+thing,
                new VArmState(VArmConstants.armLeftClipperPosition),
                new VArmState(VArmConstants.armLeftPreDepositPosition)
        );

        timeToExtendAfterDeposit = vArmToPreDepositSpikeMark.getStartTime();

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

        LinearTranslation shimmyRight = new LinearTranslation(liftToDepositSpikeMark.getEndTime(),
                new TranslationState(-2, -24-9+2),
                new TranslationState(3, -24-9+2)
        );

        ReleaseVClaw releaseVClawSpikeMark = new ReleaseVClaw(liftToDepositSpikeMark.getEndTime());

        LinearLift liftDownSpikeMark = new LinearLift(releaseVClawSpikeMark.getEndTime()+ liftDownDelay,
                new LiftState(LiftConstants.depositPosition),
                new LiftState(0)
        );






        // Plans
        LimelightPlan limelightPlan = new LimelightPlan(robot.limelightSubsystem,
                enableLimelight,
                disableLimelight
        );
        TranslationPlan translationPlan = new TranslationPlan(robot.drive, robot.localization,
                scorePreload,
                intakeClipsTranslation,
                alignClips,
                approachSpikeMark,
                splineScoreSpikeMark,
                shimmyRight
        );
        RotationPlan rotationPlan = new RotationPlan(robot.drive, robot.localization,
                rotationStill
        );
        ExtendoPlan extendoPlan = new ExtendoPlan(robot.linearSlides,
                extendToSpikeMark,
                extendoIn,
                pullSampleIn
        );
        HWristPlan h_wrist_plan = new HWristPlan(robot.horizontalIntake,
                h_wrist_align,
                h_wrist_reset
        );
        HArmPlan h_arm_plan = new HArmPlan(robot.horizontalIntake,
                h_arm_down,
                h_arm_up,
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
                looselyHoldSampleTransfer,
                grabVClaw,
                releaseVClawSpikeMark
        );

        KlipperPlan klipperPlan = new KlipperPlan(robot.clipbot,
                initKlipper,
                klipSpecimen,
                unklipSpecimen
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
                loaderRelease3,
                loaderApplyPressure
        );


        // Synchronizer
        preloadSequence = new Synchronizer(
                limelightPlan,
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




    private void initPickupMacro(ExtendoState extendoVelocity) {
        // Unpack bot pose
        Pose2d botPose = robot.localization.getPose();
        double x_bot = botPose.getX();
        double y_bot = botPose.getY();
        double heading_bot = botPose.getHeading();

        // Unpack sample pose
        Pose2d samplePose = robot.overheadSampleData.getFilteredSamplePosition(telemetry);
        double x_sample = samplePose.getX();
        double y_sample = samplePose.getY();
        double theta_sample = samplePose.getHeading();
        double theta_sample_bot = theta_sample - heading_bot + Math.PI/2.0;

        // Get extendo state
        double x_extendo = robot.linearSlides.getExtendoPosition();
        ExtendoState extendoPosition = new ExtendoState(x_extendo);
        double x_extendo_min = OverheadCameraSubsystem.CAMERA_OFFSET[0] + OverheadCameraSubsystem.CLAW_OFFSET[0];

        // Calculate target positions
        double sin = Math.sin(heading_bot);
        double cos = Math.cos(heading_bot);
        double T = (x_sample-x_bot)*sin - (y_sample-y_bot)*cos;
        double x_target_center = x_bot + T*sin;
        double y_target_center = y_bot - T*cos;
        double x_target_drive = x_target_center + y_camera*sin;
        double y_target_drive = y_target_center - y_camera*cos;

        // Get subsystem setpoints
        TranslationState translationTarget = new TranslationState(
                x_target_drive,
                y_target_drive
        );
        RotationState rotationTarget = new RotationState(
                heading_bot
        );
        ExtendoState extendoTarget = new ExtendoState(
                Math.max(0, Math.hypot(x_sample-x_target_center, y_sample-y_target_center) - x_extendo_min)
        );
        double hWristTarget = normalizeAngle(theta_sample_bot);

        telemetry.addData("[DEBUG] botPose.getHeading()", botPose.getHeading());
        telemetry.addData("[DEBUG] rotationTarget.getHeading()", rotationTarget.getHeading());
        telemetry.addData("[DEBUG] extendoTarget.getLength()", extendoTarget.getLength());
        telemetry.addData("[DEBUG] hWristTarget", hWristTarget);

        telemetry.update();

        //// SYNCHRONIZER
        // Extendo
        LinearTranslation translation = new LinearTranslation(0,
                new TranslationState(botPose),
                translationTarget
        );

        LinearRotation rotation = new LinearRotation(0,
                new RotationState(botPose),
                rotationTarget
        );


//        double previousMaxVelocity = ExtendoConstants.MAX_PATHING_VELOCITY;
//        ExtendoConstants.MAX_PATHING_VELOCITY = previousMaxVelocity / 3;
        DynamicLinearExtendo extendoOut = new DynamicLinearExtendo(0,
                extendoPosition,
                extendoTarget,
                extendoVelocity
        );
//        ExtendoConstants.MAX_PATHING_VELOCITY = previousMaxVelocity;


        // Move arm down
        LinearHArm h_arm_down = new LinearHArm(intakeDelay+Math.max(Math.max(extendoOut.getEndTime(), rotation.getEndTime()), translation.getEndTime()),
                new HArmState(0.9),
                new HArmState(armDownPosition),
                true
        );
        MoveHWrist h_wrist_align = new MoveHWrist(extendoOut.getStartTime(), hWristTarget);


        // Pick up and move arm up
        GrabHClaw h_claw_grab = new GrabHClaw(h_arm_down.getEndTime(), true);
        LinearHArm h_arm_up = new LinearHArm(Math.max(h_claw_grab.getEndTime()+intakeDelay/2, h_arm_down.getEndTime()),
                new HArmState(armDownPosition),
                new HArmState(clawCheckPosition)
        );
        LinearHArm h_arm_hold_check = new LinearHArm(new TimeSpan(h_arm_up.getEndTime(), h_arm_up.getEndTime()+0.25),
                new HArmState(clawCheckPosition),
                new HArmState(clawCheckPosition)
        );


        // Create Plans
        TranslationPlan translationPlan = new TranslationPlan(robot.drive, robot.localization,
                translation
        );
        RotationPlan rotationPlan = new RotationPlan(robot.drive, robot.localization,
                rotation
        );
        ExtendoPlan extendo_plan = new ExtendoPlan(robot.linearSlides,
                extendoOut
        );
        HWristPlan h_wrist_plan = new HWristPlan(robot.horizontalIntake,
                h_wrist_align
        );
        HArmPlan h_arm_plan = new HArmPlan(robot.horizontalIntake,
                h_arm_down,
                h_arm_up,
                h_arm_hold_check
        );
        HClawPlan h_claw_plan = new HClawPlan(robot.horizontalIntake,
                h_claw_grab
        );

        // Synchronizer
        this.pickupMacro = new Synchronizer(
                translationPlan,
                rotationPlan,
                extendo_plan,
                h_arm_plan,
                h_wrist_plan,
                h_claw_plan
        );
    }



    private void initMakerMacro() {
        MoveHWrist h_wrist_flat = new MoveHWrist(0, -Math.PI/2);

        // Lift gets ready for transfer
        LinearLift liftUp = new LinearLift(0,
                new LiftState(robot.linearSlides.getLeftLiftPosition()),
                new LiftState(LiftConstants.transferPosition)
        );

        LinearVArm prepareVArm = new LinearVArm(liftUp.getStartTime(),
                new VArmState(VArmConstants.armLeftDepositPosition),
                new VArmState(VArmConstants.armLeftPreTransferPosition)
        );


        // Retract extendo
        LinearExtendo extendoIn = new LinearExtendo(liftUp.getEndTime(),
                new ExtendoState(robot.linearSlides.getExtendoPosition()),
                new ExtendoState(0)
        );


        //// Mag has clips, do transfer and clipping sequence
        // Horizontal arm gets ready
        LinearHArm hArmUpTransfer = new LinearHArm(extendoIn.getEndTime()-horizontalArmRaiseDeadtime,
                new HArmState(clawCheckPosition),
                new HArmState(HArmConstants.armTransferPosition)
        );

        // wrist resets
        MoveHWrist h_wrist_reset = new MoveHWrist(hArmUpTransfer.getEndTime(), 0, true);

        // Move extendo to transfer position
        LinearExtendo extendoToTransfer = new LinearExtendo(hArmUpTransfer.getEndTime(),
                new ExtendoState(0),
                new ExtendoState(ExtendoConstants.transferPosition)
        );

        // Vertical arm gets ready
        LinearVArm vArmDown = new LinearVArm(Math.max(hArmUpTransfer.getEndTime(), extendoToTransfer.getEndTime()),
                new VArmState(VArmConstants.armLeftPreTransferPosition),
                new VArmState(VArmConstants.armLeftTransferPosition)
        );

        // Loosely hold sample
        MoveVClaw looselyHoldSampleTransfer = new MoveVClaw(vArmDown.getEndTime()+0.25, 0.1,
                new VClawState(VClawConstants.RELEASE_POSITION),
                new VClawState(VClawConstants.LOOSELY_GRABBED_POSITION)
        );

        // Pull sample in more
        LinearExtendo pullSampleIn = new LinearExtendo(looselyHoldSampleTransfer.getEndTime(),
                new ExtendoState(ExtendoConstants.transferPosition),
                new ExtendoState(ExtendoConstants.pullInSamplePosition)
        );

        // Deposit claw grabs sample
        MoveVClaw grabVClaw = new MoveVClaw(pullSampleIn.getEndTime(), 0.1,
                new VClawState(VClawConstants.LOOSELY_GRABBED_POSITION),
                new VClawState(VClawConstants.GRAB_POSITION)
        );

        // Intake claw releases sample
        ReleaseHClaw releaseHClaw = new ReleaseHClaw(grabVClaw.getStartTime());

        // Deposit arm moves out of the way
        LinearVArm upVArm = new LinearVArm(grabVClaw.getStartTime(),
                new VArmState(VArmConstants.armLeftTransferPosition),
                new VArmState(VArmConstants.armLeftTransferPosition-0.1)
        );

        // Intake arm moves back down
        LinearHArm hArmDown = new LinearHArm(Math.max(releaseHClaw.getEndTime(), hArmUpTransfer.getEndTime()),
                new HArmState(HArmConstants.armTransferPosition),
                new HArmState(0.9)
        );

        //// CLIPBOT
        // Get target states
        double maxClips = MFeederConstants.MAX_CAPACITY;
        MFeederState currentFeederPosition = new MFeederState(
                robot.clipbot.getMagazineFeederPosition()
        );
        MFeederState targetFeederPosition = new MFeederState(
                (maxClips - (clipInventory-1)) * MFeederConstants.INCHES_PER_CLIP
        );

        /// Movements
        // Hold lift down
        LinearLift holdLiftDown = new LinearLift(hArmDown.getEndTime() - verticalLiftDownDeadtime,
                new LiftState(LiftConstants.transferPosition),
                new LiftState(holdLiftDownPosition)
        );

        // Stationary deposit arm
        LinearVArm lowerVArm = new LinearVArm(holdLiftDown.getEndTime()-0.38,
                new VArmState(VArmConstants.armLeftTransferPosition-0.1),
                new VArmState(VArmConstants.armLeftClipperPosition)
        );

        // Stationary deposit arm
        LinearVArm pressVArm = new LinearVArm(holdLiftDown.getEndTime(),
                new VArmState(VArmConstants.armLeftClipperPosition),
                new VArmState(VArmConstants.armLeftClipperPosition)
        );

        // Advance feeder by one clip
        LinearMFeeder advanceFeeder;


        // Feeder plan
        MFeederPlan mFeederPlan;
        if (clipInventory-1==0) {
            double ti = holdLiftDown.getEndTime() + feederDelayTime;
            advanceFeeder = new LinearMFeeder(new TimeSpan(ti, ti+advanceTime),
                    currentFeederPosition,
                    new MFeederState(targetFeederPosition.getPosition()+0.1)
            );
            LinearMFeeder resetFeeder = new LinearMFeeder(advanceFeeder.getEndTime(),
                    new MFeederState(targetFeederPosition.getPosition()+0.1),
                    new MFeederState(-3*MFeederConstants.ZERO_HOME)
            );
            mFeederPlan = new MFeederPlan(robot.clipbot,
                    advanceFeeder,
                    resetFeeder
            );
            inventoryStocked = false;
        } else {
            double ti = holdLiftDown.getEndTime() + feederDelayTime;
            advanceFeeder = new LinearMFeeder(new TimeSpan(ti, ti+advanceTime),
                    currentFeederPosition,
                    targetFeederPosition
            );
            mFeederPlan = new MFeederPlan(robot.clipbot,
                    advanceFeeder
            );
        }

        MoveMLoader initLoader = new MoveMLoader(0,
                new MLoaderState(loaderPressurePosition)
        );

        MoveMLoader loosenLoader = new MoveMLoader(advanceFeeder.getStartTime()-0.25,
                new MLoaderState(loaderFeedingPosition)
        );

        MoveMLoader tightenLoader = new MoveMLoader(advanceFeeder.getEndTime()+0.25,
                new MLoaderState(loaderPressurePosition)
        );

        // Klipper action
        MoveKlipper initKlipper = new MoveKlipper(0, KlipperConstants.openPosition);
        MoveKlipper klipSpecimen = new MoveKlipper(advanceFeeder.getEndTime()+klipperWaitTime, KlipperConstants.closedPosition);
        MoveKlipper unklipSpecimen = new MoveKlipper(klipSpecimen.getEndTime(), 0.7);













        // Create Plans
        ExtendoPlan extendo_plan = new ExtendoPlan(robot.linearSlides,
                extendoIn,
                extendoToTransfer,
                pullSampleIn
        );
        HWristPlan h_wrist_plan = new HWristPlan(robot.horizontalIntake,
                h_wrist_flat,
                h_wrist_reset
        );
        HArmPlan h_arm_plan = new HArmPlan(robot.horizontalIntake,
                hArmUpTransfer,
                hArmDown
        );
        HClawPlan h_claw_plan = new HClawPlan(robot.horizontalIntake,
                releaseHClaw
        );
        LiftPlan liftPlan = new LiftPlan(robot.linearSlides,
                liftUp,
                holdLiftDown
        );
        VArmPlan vArmPlan = new VArmPlan(robot.verticalDeposit,
                prepareVArm,
                vArmDown,
                upVArm,
                lowerVArm,
                pressVArm
        );
        VClawPlan vClawPlan = new VClawPlan(robot.verticalDeposit,
                looselyHoldSampleTransfer,
                grabVClaw
        );

        MLoaderPlan mLoaderPlan = new MLoaderPlan(robot.clipbot,
                initLoader,
                loosenLoader,
                tightenLoader
        );

        KlipperPlan klipperPlan = new KlipperPlan(robot.clipbot,
                initKlipper,
                klipSpecimen,
                unklipSpecimen
        );

        // Synchronizer
        this.makerMacro = new Synchronizer(
                extendo_plan,
                h_arm_plan,
                h_wrist_plan,
                h_claw_plan,
                liftPlan,
                vArmPlan,
                vClawPlan,
                mFeederPlan,
                mLoaderPlan,
                klipperPlan
        );

    }




    double depositStartDelay = 0.5;
    private void initDepositMacro() {

        // Score specimen
        LinearVArm vArmToPreDepositCycle = new LinearVArm(depositStartDelay,
                new VArmState(VArmConstants.armLeftClipperPosition),
                new VArmState(VArmConstants.armLeftPreDepositPosition)
        );

        LinearLift liftToPreDepositCycle = new LinearLift(vArmToPreDepositCycle.getStartTime(),
                new LiftState(0),
                new LiftState(LiftConstants.preDepositPosition)
        );

        LinearVArm vArmToDepositCycle = new LinearVArm(liftToPreDepositCycle.getEndTime(),
                new VArmState(VArmConstants.armLeftPreDepositPosition),
                new VArmState(VArmConstants.armLeftDepositPosition)
        );

        LinearLift liftToDepositCycle = new LinearLift(vArmToDepositCycle.getEndTime()+0.1,
                new LiftState(LiftConstants.preDepositPosition),
                new LiftState(LiftConstants.depositPosition)
        );



        // Create Plans
        LiftPlan liftPlan = new LiftPlan(robot.linearSlides,
                liftToPreDepositCycle,
                liftToDepositCycle
        );
        VArmPlan vArmPlan = new VArmPlan(robot.verticalDeposit,
                vArmToPreDepositCycle,
                vArmToDepositCycle
        );


        // Synchronizer
        this.depositMacro = new Synchronizer(
                liftPlan,
                vArmPlan
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
