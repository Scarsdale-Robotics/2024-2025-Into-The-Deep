package org.firstinspires.ftc.teamcode.opmodes.calibration.combined_testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.cvprocessors.SampleOrientationProcessor;
import org.firstinspires.ftc.teamcode.opmodes.algorithms.SampleDataBufferFilter;
import org.firstinspires.ftc.teamcode.opmodes.calibration.Drawing;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LocalizationSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.AutonomousRobot;
import org.firstinspires.ftc.teamcode.synchropather.macros.EducatedSearchMacro;
import org.firstinspires.ftc.teamcode.synchropather.macros.ExtendoRetractMacro;
import org.firstinspires.ftc.teamcode.synchropather.macros.SearchMacro;
import org.firstinspires.ftc.teamcode.synchropather.subsystemclasses.HorizontalIntakeSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.subsystemclasses.LinearSlidesSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.subsystemclasses.OverheadCameraSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.systems.MovementType;
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
import org.firstinspires.ftc.teamcode.synchropather.systems.limelight.movements.EnableLimelight;
import org.firstinspires.ftc.teamcode.synchropather.systems.mFeeder.MFeederConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.mFeeder.MFeederPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.mFeeder.MFeederState;
import org.firstinspires.ftc.teamcode.synchropather.systems.mFeeder.movements.LinearMFeeder;
import org.firstinspires.ftc.teamcode.synchropather.systems.mLoader.MLoaderConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.rotation.RotationPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.rotation.RotationState;
import org.firstinspires.ftc.teamcode.synchropather.systems.rotation.movements.LinearRotation;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.TranslationPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.TranslationState;
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

import java.util.ArrayDeque;
import java.util.List;

@Config
@TeleOp(name="Claw Vacancy Test", group = "Calibration")
public class ClawVacancyTest extends LinearOpMode {

    private Synchronizer search, pickup;
    private Synchronizer extendoRetract;

    private AutonomousRobot robot;
    private ArrayDeque<Double> loopTicks;
    private ElapsedTime runtime;

    private HorizontalIntakeSubsystem horizontalIntake;
    private OverheadCameraSubsystem overheadCamera;
    private LinearSlidesSubsystem linearSlides;
    private LocalizationSubsystem localization;
    private DriveSubsystem drive;

    private SampleDataBufferFilter sampleData;

    public static double armDownPosition = 1.025;

    public static double driveSpeed = 1;
    public static int overheadFilterLength = 7;
    public static double intakeDelay = 0.25;


    // For clipbot subsystem
    private int clipInventory = MFeederConstants.RELOAD_CAPACITY;
    private boolean inventoryStocked = true;

    public static double klipperWaitTime = 0.3;
    public static double feederDelayTime = -0.09;
    public static double holdLiftDownPosition = -1;



    // Deposit
    public static double liftUpDepositDelay = 0.25;
    public static double liftDownDelay = 0.1;



    // Checking if claw picked up anything
    private double pickupCheckClawTime;
    public static double clawCheckPosition = 0.79;
    public static double checkPixelsDecayFactor = 1;
    public static double pickupCheckClawTimeDelay = 0.25;


    // loader applying pressure on clips
    public static double loaderPressurePosition = 0.075;



    // Optimization
    public static double horizontalArmRaiseDeadtime = 0.43;
    public static double verticalLiftDownDeadtime = 0.23;


    public static double y_camera = 0;//-0.57047244;



    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        loopTicks = new ArrayDeque<>();
        runtime = new ElapsedTime(0);
        runtime.reset();

        telemetry.addData("[MAIN] TPS", 0);
        telemetry.update();

        waitForStart();


        EnableLimelight enableLimelight = new EnableLimelight(new TimeSpan(0,10), LimelightPipeline.SAMPLE_DETECTOR);
        LimelightPlan limelightPlan = new LimelightPlan(robot.limelightSubsystem, enableLimelight);
        Synchronizer limelightAction = new Synchronizer(limelightPlan);
        limelightAction.start();
        limelightAction.update();

        boolean toggleTriangle = false;
        boolean sampleMacroRunning = false;
        boolean clawGrabbed = false;
        boolean previousDriverControlling = true;
        boolean clawHasSomething = false;
        ExtendoState extendoVelocity = null;
        double checkPixels = 0;
        while (opModeIsActive()) {
            robot.update();
            updateTPS();
            boolean driverControlling = controlDrive(sampleMacroRunning, previousDriverControlling);
            previousDriverControlling = driverControlling;


            // Telemetry for claw vacancy processor
            int clawPixelCount = robot.clawVacancyProcessor.getPixelCount();
            boolean clawEmpty = robot.clawVacancyProcessor.isClawEmpty();
            telemetry.addData("clawPixelCount", clawPixelCount);
            telemetry.addData("clawEmpty", clawEmpty);


            if (gamepad1.cross) {
                robot.clipbot.setMagazineLoaderPosition(MLoaderConstants.openPosition);
            } else {
                robot.clipbot.setMagazineLoaderPosition(loaderPressurePosition);
            }




            // Restock option if empty
            if (!inventoryStocked && gamepad1.square) {
                inventoryStocked = true;
                clipInventory = MFeederConstants.RELOAD_CAPACITY;
            }


            // Look for limelight samples
            Pose2d botPose = localization.getPose();
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
                    double distance = Math.hypot(samplePosition[0]-botPose.getX(), samplePosition[1]-botPose.getY());
                    if (distance < closestDistance) {
                        closestDistance = distance;
                        closestSample = samplePosition;
                    }
                }
                foundSample = closestSample;
            }

            handleGamepadColor();

            //// Triangle button
            // Case: Init search macro
            if (gamepad1.triangle && !toggleTriangle && !sampleMacroRunning && !clawGrabbed) {
                drive.stopController();
                if (foundSample!=null) {
                    // init search
                    search = new EducatedSearchMacro(
                            foundSample,
                            robot,
                            1
                    );
                    sampleData.setFilterLength(7);
                } else {
                    // blind search
                    search = new SearchMacro(
                            ExtendoConstants.MAX_EXTENSION-5,
                            robot.linearSlides,
                            robot.horizontalIntake
                    );
                    sampleData.setFilterLength(3);
                }
                search.start();
                sampleData.clearFilterData();
                sampleMacroRunning = true;
                clawHasSomething = false;
            }
            // Case: Drop off sample
            else if (gamepad1.triangle && !toggleTriangle && !sampleMacroRunning && clawGrabbed) {
                horizontalIntake.setClawPosition(HClawConstants.RELEASE_POSITION);
                clawGrabbed = false;
            }
            // Case: Cancel search macro
            else if (gamepad1.triangle && !toggleTriangle && sampleMacroRunning) {
                sampleMacroRunning = false;
                if (sampleData.isFilterFull()) {
                    clipInventory++;
                    inventoryStocked = true;
                }
            }
            toggleTriangle = gamepad1.triangle;


            //// Sample macro control
            if (sampleMacroRunning) {
                // Search motion
                if (!sampleData.isFilterFull()) {
                    boolean searchRunning = search.update();
                    // Did not find sample
                    if (!searchRunning) {
                        search.stop();
                        sampleMacroRunning = false;
                    }
                    // Try to detect sample
                    else {
                        sampleData.updateFilterData(overheadCamera.getSamplePositions(), overheadCamera.getSampleAngles(), overheadCamera.getClosestSample()); // Can return null
                        extendoVelocity = (ExtendoState) search.getVelocity(MovementType.EXTENDO);
                    }
                }
                // Pickup motion
                else {
                    // Init pickup
                    if ((pickup==null || !pickup.getIsRunning()) || search.getIsRunning()) {
                        search.stop();
                        initPickupMotion(extendoVelocity);
                        pickup.start();
                    }
                    boolean bad = false;
                    // Check if claw has grabbed the sample
                    if (pickup.getElapsedTime()>pickupCheckClawTime && !clawHasSomething) {
                        checkPixels = robot.clawVacancyProcessor.getPixelCount();
                        telemetry.addData("ONE TIME checkPixels", checkPixels);
                        clawHasSomething = !robot.clawVacancyProcessor.isClawEmpty();
                        bad = !clawHasSomething;
                    }
                    telemetry.addData("pickup.getElapsedTime()", pickup.getElapsedTime());
                    // Stop macro if driver took over (or macro ended)
                    if (driverControlling || (pickup.getIsRunning() && !pickup.update()) || bad) {
                        // re compensate mag
                        if (pickup.getElapsedTime()>pickupCheckClawTime && !clawHasSomething) {
                            clipInventory++;
                            inventoryStocked = true;
                        }
                        pickup.stop();
                        sampleMacroRunning = false;
                        clawGrabbed = true;
                        // init extendo retract macro
                        extendoRetract = new ExtendoRetractMacro(linearSlides);
                        extendoRetract.start();
                    }
                }
            }
            else {
                extendoRetract.update();
            }

            telemetry.addData("sampleData.getFilterLength()", sampleData.getFilterLength());
            telemetry.addData("CHECK clawPixelCount", checkPixels);
            checkPixels *= checkPixelsDecayFactor;

            telemetry.addData("pickupCheckClawTime", pickupCheckClawTime);
            telemetry.addData("sampleMacroRunning", sampleMacroRunning);


            if (gamepad1.cross) {
                //// Draw detected sample position
                TelemetryPacket packet = new TelemetryPacket();
                packet.fieldOverlay().setStroke("#3F51B5");
                Drawing.drawRobot(packet.fieldOverlay(), localization.getPose());
                Pose2d samplePosition = sampleData.getFilteredSamplePosition(telemetry);
                if (samplePosition != null) {
                    telemetry.addData("samplePosition", samplePosition.toString());
                    Drawing.drawSample(packet.fieldOverlay(), samplePosition, "#FF0000");
                }
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
            }


            telemetry.update();
        }
    }


    /**
     * Field centric drive (based on driver POV)
     * @return whether or not the driver is giving joystick inputs
     */
    private boolean controlDrive(boolean macroRunning, boolean previousDriving) {
        double forward = gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;
        boolean driving = !(forward==0 && strafe==0 && turn==0);

        if (!driving && !previousDriving) return false;

        if (driving ||
                !macroRunning || // [!driving] braking allowed when macro is deactivated
                !sampleData.isFilterFull() // [!driving && macroRunning] braking allowed during search
        ) {
            drive.driveFieldCentricPowers(
                    driveSpeed * forward,
                    driveSpeed * strafe,
                    Math.min(1,driveSpeed+0.5) * turn,
                    Math.toDegrees(localization.getH())
            );
        }

        return driving;
    }


    /**
     * Sets the gamepad led color to the cv sample color
     */
    private void handleGamepadColor() {
        if (robot.teamColor == AutonomousRobot.TeamColor.BLUE) {
            SampleOrientationProcessor.colorType = SampleOrientationProcessor.SampleColor.BLUE;
        }
        else {
            SampleOrientationProcessor.colorType = SampleOrientationProcessor.SampleColor.RED;
        }

        switch (SampleOrientationProcessor.colorType) {
            case YELLOW:
                gamepad1.setLedColor(1,1,0,1000);
                break;
            case BLUE:
                gamepad1.setLedColor(0,0,1,1000);
                break;
            case RED:
                gamepad1.setLedColor(1,0,0,1000);
                break;
            default:
                break;
        }
    }


    private void initialize() {
        // init subsystems
        this.robot = new AutonomousRobot(
                hardwareMap,
                new Pose2d(1,1,new Rotation2d(Math.PI/2)),
                AutonomousRobot.TeamColor.RED,
                this,
                SampleDataBufferFilter.SampleTargetingMethod.TRANSLATION
        );
        this.telemetry = robot.telemetry;
        this.horizontalIntake = robot.horizontalIntake;
        this.overheadCamera = robot.overheadCamera;
        this.linearSlides = robot.linearSlides;
        this.localization = robot.localization;
        this.drive = robot.drive;
        robot.setOverheadSampleDataBufferFilter(
                new SampleDataBufferFilter(
                        linearSlides,
                        localization,
                        0.04375,
                        overheadFilterLength,
                        robot.sampleTargetingMethod
                )
        );
        this.sampleData = robot.overheadSampleData;
        OverheadCameraSubsystem.CLAW_OFFSET[0] = -3;
        SampleDataBufferFilter.FILTER_ERROR_TOLERANCE = 0.15;

        robot.verticalDeposit.setArmPosition(0.5);
        robot.verticalDeposit.setClawPosition(VClawConstants.RELEASE_POSITION);
        robot.horizontalIntake.setArmPosition(0.9);
        robot.horizontalIntake.setWristAngle(0);
        robot.horizontalIntake.setClawPosition(HClawConstants.GRAB_POSITION);

        // init extendo retract macro
        extendoRetract = new ExtendoRetractMacro(linearSlides);
        extendoRetract.start();

        // init servos
        horizontalIntake.setClawPosition(HClawConstants.RELEASE_POSITION);
        horizontalIntake.setWristAngle(0);
        horizontalIntake.setArmPosition(0.9);

        // pressure on clips
        robot.clipbot.setMagazineLoaderPosition(loaderPressurePosition);
    }

    private void updateTPS() {
        // TPS counter
        double currentTime = runtime.seconds();
        loopTicks.add(currentTime);
        while (!loopTicks.isEmpty() && currentTime - loopTicks.getFirst() > 1d) loopTicks.removeFirst();
        telemetry.addData("[MAIN] TPS", loopTicks.size());
        telemetry.update();
    }


    private void initPickupMotion(ExtendoState extendoVelocity) {
        // Unpack bot pose
        Pose2d botPose = localization.getPose();
        double x_bot = botPose.getX();
        double y_bot = botPose.getY();
        double heading_bot = botPose.getHeading();

        // Unpack sample pose
        Pose2d samplePose = sampleData.getFilteredSamplePosition(telemetry);
        double x_sample = samplePose.getX();
        double y_sample = samplePose.getY();
        double theta_sample = samplePose.getHeading();
        double theta_sample_bot = theta_sample - heading_bot + Math.PI/2.0;

        // Get extendo state
        double x_extendo = linearSlides.getExtendoPosition();
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

        // Lift gets ready for transfer
        LinearLift liftUp = new LinearLift(extendoOut.getStartTime(),
                new LiftState(robot.linearSlides.getLeftLiftPosition()),
                new LiftState(LiftConstants.transferPosition)
        );

        LinearVArm prepareVArm = new LinearVArm(liftUp.getStartTime(),
                new VArmState(VArmConstants.armLeftDepositPosition),
                new VArmState(VArmConstants.armLeftPreTransferPosition)
        );

        // Move arm down
        LinearHArm h_arm_down = new LinearHArm(intakeDelay+Math.max(Math.max(extendoOut.getEndTime(), rotation.getEndTime()), translation.getEndTime()),
                new HArmState(0.9),
                new HArmState(armDownPosition),
                true
        );
        MoveHWrist h_wrist_align = new MoveHWrist(extendoOut.getStartTime(), hWristTarget);

        // Pick up and move arm up
        GrabHClaw h_claw_grab = new GrabHClaw(h_arm_down.getEndTime()+intakeDelay/2, true);
        LinearHArm h_arm_up = new LinearHArm(Math.max(h_claw_grab.getEndTime(), h_arm_down.getEndTime()),
                new HArmState(armDownPosition),
                new HArmState(clawCheckPosition)
        );
        MoveHWrist h_wrist_flat = new MoveHWrist(h_arm_up.getEndTime(), -Math.PI/2, true);

        pickupCheckClawTime = h_wrist_flat.getEndTime()+pickupCheckClawTimeDelay;


        // Retract extendo
        LinearExtendo extendoIn = new LinearExtendo(h_wrist_flat.getStartTime(),
                extendoTarget,
                new ExtendoState(0)
        );

        // Only pick up if mag is empty
        if (!inventoryStocked || clipInventory <= 0) {
            // Create Plans
            TranslationPlan translationPlan = new TranslationPlan(drive, localization,
                    translation
            );
            RotationPlan rotationPlan = new RotationPlan(drive, localization,
                    rotation
            );
            ExtendoPlan extendo_plan = new ExtendoPlan(linearSlides,
                    extendoOut,
                    extendoIn
            );
            HWristPlan h_wrist_plan = new HWristPlan(horizontalIntake,
                    h_wrist_align,
                    h_wrist_flat
            );
            HArmPlan h_arm_plan = new HArmPlan(horizontalIntake,
                    h_arm_down,
                    h_arm_up
            );
            HClawPlan h_claw_plan = new HClawPlan(horizontalIntake,
                    h_claw_grab
            );

            // Synchronizer
            this.pickup = new Synchronizer(
                    translationPlan,
                    rotationPlan,
                    extendo_plan,
                    h_arm_plan,
                    h_wrist_plan,
                    h_claw_plan
            );
            return;
        }


        //// Mag has clips, do transfer and clipping sequence
        // Horizontal arm gets ready
        LinearHArm hArmUpTransfer = new LinearHArm(Math.max(extendoIn.getEndTime()-horizontalArmRaiseDeadtime, h_arm_up.getEndTime()),
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

        clipInventory--;
        MFeederState targetFeederPosition = new MFeederState(
                (maxClips - clipInventory) * MFeederConstants.INCHES_PER_CLIP
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
                new VArmState(VArmConstants.armLeftClipperPosition+0.02),
                new VArmState(VArmConstants.armLeftClipperPosition+0.02)
        );

        // Advance feeder by one clip
        LinearMFeeder advanceFeeder = new LinearMFeeder(holdLiftDown.getEndTime() + feederDelayTime,
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
        LinearVArm vArmToPreDepositCycle = new LinearVArm(unklipSpecimen.getStartTime()+liftUpDepositDelay,
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

        ReleaseVClaw releaseVClawCycle = new ReleaseVClaw(liftToDepositCycle.getEndTime());

        LinearLift liftDownCycle = new LinearLift(releaseVClawCycle.getEndTime()+ liftDownDelay,
                new LiftState(LiftConstants.depositPosition),
                new LiftState(0)
        );










        // Create Plans
        TranslationPlan translationPlan = new TranslationPlan(drive, localization,
                translation
        );
        RotationPlan rotationPlan = new RotationPlan(drive, localization,
                rotation
        );
        ExtendoPlan extendo_plan = new ExtendoPlan(linearSlides,
                extendoOut,
                extendoIn,
                extendoToTransfer,
                pullSampleIn
        );
        HWristPlan h_wrist_plan = new HWristPlan(horizontalIntake,
                h_wrist_align,
                h_wrist_flat,
                h_wrist_reset
        );
        HArmPlan h_arm_plan = new HArmPlan(horizontalIntake,
                h_arm_down,
                h_arm_up,
                hArmUpTransfer,
                hArmDown
        );
        HClawPlan h_claw_plan = new HClawPlan(horizontalIntake,
                h_claw_grab,
                releaseHClaw
        );
        LiftPlan liftPlan = new LiftPlan(robot.linearSlides,
                liftUp,
                holdLiftDown,
                liftToPreDepositCycle,
                liftToDepositCycle,
                liftDownCycle
        );
        VArmPlan vArmPlan = new VArmPlan(robot.verticalDeposit,
                prepareVArm,
                vArmDown,
                upVArm,
                lowerVArm,
                pressVArm,
                vArmToPreDepositCycle,
                vArmToDepositCycle
        );
        VClawPlan vClawPlan = new VClawPlan(robot.verticalDeposit,
                looselyHoldSampleTransfer,
                grabVClaw,
                releaseVClawCycle
        );

        KlipperPlan klipperPlan = new KlipperPlan(robot.clipbot,
                initKlipper,
                klipSpecimen,
                unklipSpecimen
        );

        // Synchronizer
        this.pickup = new Synchronizer(
                translationPlan,
                rotationPlan,
                extendo_plan,
                h_arm_plan,
                h_wrist_plan,
                h_claw_plan,
                liftPlan,
                vArmPlan,
                vClawPlan,
                mFeederPlan,
                klipperPlan
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
