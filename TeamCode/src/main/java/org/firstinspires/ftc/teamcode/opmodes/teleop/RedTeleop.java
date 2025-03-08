package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.cvprocessors.SampleOrientationProcessor;
import org.firstinspires.ftc.teamcode.opmodes.algorithms.SampleDataBufferFilter;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LocalizationSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.AutonomousRobot;
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
import org.firstinspires.ftc.teamcode.synchropather.systems.hWrist.HWristState;
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
import org.firstinspires.ftc.teamcode.synchropather.systems.mIntake.MIntakeConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.mLoader.MLoaderConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.mLoader.MLoaderPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.mLoader.MLoaderState;
import org.firstinspires.ftc.teamcode.synchropather.systems.mLoader.movements.MoveMLoader;
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

import java.util.ArrayDeque;

@Config
@Disabled
@TeleOp(name="Red Teleop")
public class RedTeleop extends LinearOpMode {

    private Synchronizer pickupMacro, makerMacro, depositMacro;

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

    public static double gamepad1DriveSpeed = 1;

    public static double gamepad2DriveSpeed = 0.5;
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
    public static double loaderPressurePosition = 0.02;
    public static double loaderFeedingPosition = 0.05;



    // Optimization
    public static double horizontalArmRaiseDeadtime = 0.43;
    public static double verticalLiftDownDeadtime = 0.23;


    // Intake
    public static double y_camera = 0;//-0.57047244;


    // Driver Controls
    private boolean gamepad1HasDriveControl = true;

    boolean depositReadyToRelease = false;

    boolean intakeMacroRunning = false;
    boolean intookSample = false;
    boolean toggleTriangleG2 = false;

    boolean makerMacroRunning = false;
    boolean specimenMade = false;
    boolean toggleSquareG2 = false;

    boolean depositMacroRunning = false;
    boolean deposited = true;
    boolean toggleTriangleG1 = false;

    double lastLiftPower = 0;
    double lastExtendoPower = 0;


    // magazine mode
    private Synchronizer magazineRepositionMacro;
    private boolean magazineRepositionMacroRunning = false;
    private boolean toggleGamepad2DpadLeft = false;
    private boolean toggleGamepad2DpadRight = false;
    private double magazineIntakePosition = 0;
    public static double magazineIntakeDescentSpeed = 0.01;



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



        boolean previousDriverControlling = true;
        while (opModeIsActive()) {
            robot.update();
            updateTPS();
            boolean driverControlling = controlDrive(intakeMacroRunning, previousDriverControlling);
            previousDriverControlling = driverControlling;


            /// Claim drive control by pressing both joysticks
            if (gamepad1.left_stick_button && gamepad1.right_stick_button) {
                gamepad1HasDriveControl = true;
            } else if (gamepad2.left_stick_button && gamepad2.right_stick_button) {
                gamepad1HasDriveControl = false;
            }



            if (gamepad1HasDriveControl) {
                handleMacroMode();
            } else {
                handleMagazineMode();
            }


            handleGamepadColors();

            telemetry.update();

        }
    }


    /**
     * Field centric drive (based on driver POV)
     * @return whether or not the driver is giving joystick inputs
     */
    private boolean controlDrive(boolean macroRunning, boolean previousDriving) {
        // macro mode controls
        if (gamepad1HasDriveControl) {
            double forward = gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;
            boolean driving = !(forward == 0 && strafe == 0 && turn == 0);

            if (!driving && !previousDriving) return false;

            if (!macroRunning) {
                drive.driveFieldCentricPowers(
                        gamepad1DriveSpeed * forward,
                        gamepad1DriveSpeed * strafe,
                        Math.min(1, gamepad1DriveSpeed + 0.5) * turn,
                        Math.toDegrees(localization.getH())
                );
            }

            return driving;
        }
        // mag mode controls
        else {
            double forward = gamepad2.left_stick_y;
            double strafe = gamepad2.left_stick_x;
            double turn = gamepad2.right_stick_x;

            drive.driveFieldCentricPowers(
                    gamepad2DriveSpeed * forward,
                    gamepad2DriveSpeed * strafe,
                    gamepad2DriveSpeed/2 * turn,
                    Math.toDegrees(localization.getH())
            );

            return false;
        }
    }


    private void handleMagazineMode() {
        /// Set restocked to true
        inventoryStocked = true;

        /// Magazine controls
        // Control feeder
        if (gamepad2.left_bumper) {
            if (!magazineRepositionMacroRunning) {
                double totalPower = gamepad2.right_trigger - gamepad2.left_trigger;
                robot.clipbot.setMagazineFeederPower(totalPower);
            }
        }
        // Control servos
        else {
            double desiredMagazineIntakePosition = MIntakeConstants.closedPosition + gamepad2.left_trigger * (MIntakeConstants.upPosition - MIntakeConstants.closedPosition);
            if (desiredMagazineIntakePosition < magazineIntakePosition) {
                magazineIntakePosition = Math.max(desiredMagazineIntakePosition, magazineIntakePosition-magazineIntakeDescentSpeed);
            } else {
                magazineIntakePosition = desiredMagazineIntakePosition;
            }
            robot.clipbot.setMagazineIntakePosition(magazineIntakePosition);
            double magazineLoaderPosition = MLoaderConstants.openPosition + gamepad2.right_trigger * (MLoaderConstants.maxClosedPosition - MLoaderConstants.openPosition);
            robot.clipbot.setMagazineLoaderPosition(magazineLoaderPosition);
            if (!magazineRepositionMacroRunning) {
                robot.clipbot.setMagazineFeederPower(0);
            }
        }

        /// Magazine reposition macro
        // dpad left is increase capacity
        if (gamepad2.dpad_left && !toggleGamepad2DpadLeft && !magazineRepositionMacroRunning && clipInventory < MFeederConstants.MAX_CAPACITY) {
            int maxClips = MFeederConstants.MAX_CAPACITY;
            MFeederState currentFeederPosition = new MFeederState(
                    robot.clipbot.getMagazineFeederPosition()
            );
            MFeederState targetFeederPosition = new MFeederState(
                    (maxClips - (clipInventory+1)) * MFeederConstants.INCHES_PER_CLIP
            );
            LinearMFeeder moveFeeder = new LinearMFeeder(0, currentFeederPosition, targetFeederPosition);
            MFeederPlan mFeederPlan = new MFeederPlan(robot.clipbot, moveFeeder);
            magazineRepositionMacro = new Synchronizer(mFeederPlan);
            magazineRepositionMacro.start();
            magazineRepositionMacroRunning = true;
            toggleGamepad2DpadLeft = true;
            clipInventory++;
        }
        if (!gamepad2.dpad_left) toggleGamepad2DpadLeft = false;

        // dpad right is decrease capacity
        if (gamepad2.dpad_right && !toggleGamepad2DpadRight && !magazineRepositionMacroRunning && clipInventory > 0) {
            int maxClips = MFeederConstants.MAX_CAPACITY;
            MFeederState currentFeederPosition = new MFeederState(
                    robot.clipbot.getMagazineFeederPosition()
            );
            MFeederState targetFeederPosition = new MFeederState(
                    (maxClips - (clipInventory-1)) * MFeederConstants.INCHES_PER_CLIP
            );
            LinearMFeeder moveFeeder = new LinearMFeeder(0, currentFeederPosition, targetFeederPosition);
            MFeederPlan mFeederPlan = new MFeederPlan(robot.clipbot, moveFeeder);
            magazineRepositionMacro = new Synchronizer(mFeederPlan);
            magazineRepositionMacro.start();
            magazineRepositionMacroRunning = true;
            toggleGamepad2DpadRight = true;
            clipInventory--;
        }
        if (!gamepad2.dpad_right) toggleGamepad2DpadRight = false;


        // control macro
        if (magazineRepositionMacroRunning) {
            if (!magazineRepositionMacro.update()) {
                magazineRepositionMacro.stop();
                magazineRepositionMacroRunning = false;
            }
        }
    }

    private void handleMacroMode() {

        /// Mag default position applies pressure on clips
        if (!makerMacroRunning) {
            robot.clipbot.setMagazineLoaderPosition(loaderPressurePosition);
        }

        /// gamepad 2 manual control extendo and lift
        double totalSecondGamepadTrigger = gamepad2.right_trigger - gamepad2.left_trigger;
        // left bumper = lift control
        if (gamepad2.left_bumper) {
            if ((totalSecondGamepadTrigger!=0 || lastLiftPower!=0) && !(makerMacroRunning || depositMacroRunning)) {
                robot.linearSlides.setLiftPowers(totalSecondGamepadTrigger);
                lastLiftPower = totalSecondGamepadTrigger;
            }
            if (!(intakeMacroRunning || makerMacroRunning)) {
                robot.linearSlides.setExtendoPower(0);
                lastExtendoPower = 0;
            }
        }
        // no bumper = extendo control
        else {
            if ((totalSecondGamepadTrigger!=0 || lastExtendoPower!=0) && !(intakeMacroRunning || makerMacroRunning)) {
                robot.linearSlides.setExtendoPower(totalSecondGamepadTrigger);
                lastExtendoPower = totalSecondGamepadTrigger;
            }
            if (!(makerMacroRunning || depositMacroRunning)) {
                robot.linearSlides.setLiftPowers(0);
                lastLiftPower = 0;
            }
        }


        /// update sample data from overhead camera
        sampleData.updateFilterData(overheadCamera.getSamplePositions(), overheadCamera.getSampleAngles(), overheadCamera.getClosestSample()); // Can return null



        /// gamepad 2 triangle activates pickup
        if (gamepad2.triangle && !toggleTriangleG2 && sampleData.isFilterFull() && !intakeMacroRunning) {
            initPickupMacro(new ExtendoState(0));
            pickupMacro.start();
            toggleTriangleG2 = true;
            intakeMacroRunning = true;
            intookSample = false;
        }
        // run synchronizer
        if (intakeMacroRunning) {
            if (!pickupMacro.update()) {
                intakeMacroRunning = false;
                intookSample = true;
                pickupMacro.stop();
            } else {
                HWristState wristState = (HWristState) pickupMacro.getState(MovementType.HORIZONTAL_WRIST);
                telemetry.addData("wristState", wristState);
            }
        }
        // second press cancels it
        if (gamepad2.triangle && !toggleTriangleG2 && intakeMacroRunning) {
            intakeMacroRunning = false;
            toggleTriangleG2 = true;
            intookSample = false;
            pickupMacro.stop();
            robot.horizontalIntake.setClawPosition(HClawConstants.RELEASE_POSITION);
            robot.horizontalIntake.setArmPosition(clawCheckPosition);
            robot.horizontalIntake.setWristAngle(0);
        }
        if (gamepad2.triangle && !toggleTriangleG2) {
            robot.horizontalIntake.setClawPosition(HClawConstants.RELEASE_POSITION);
            robot.horizontalIntake.setArmPosition(clawCheckPosition);
            robot.horizontalIntake.setWristAngle(0);
        }
        if (!gamepad2.triangle) toggleTriangleG2 = false;




        /// gamepad 2 square activates maker macro
        if (gamepad2.square && !toggleSquareG2 && intookSample && !makerMacroRunning && deposited && inventoryStocked && clipInventory>0) {
            initMakerMacro();
            makerMacro.start();
            toggleSquareG2 = true;
            makerMacroRunning = true;
            specimenMade = false;
        }
        // run synchronizer
        if (makerMacroRunning) {
            if (!makerMacro.update()) {
                makerMacroRunning = false;
                specimenMade = true;
                clipInventory--;
                if (clipInventory==0) {
                    inventoryStocked = false;
                    // clipinventory goes back to max, but inventorystocked remains false
                    clipInventory = MFeederConstants.RELOAD_CAPACITY;
                }
                intookSample = false;
                robot.clipbot.setMagazineFeederPower(0);
                robot.linearSlides.stopLifts();
                robot.linearSlides.stopExtendo();
            } else {
                HWristState wristState = (HWristState) makerMacro.getState(MovementType.HORIZONTAL_WRIST);
                telemetry.addData("wristState", wristState);
            }
        }
        // second press cancels it
        if (gamepad2.square && !toggleSquareG2 && makerMacroRunning) {
            robot.clipbot.setMagazineFeederPower(0);
            robot.linearSlides.stopLifts();
            robot.linearSlides.stopExtendo();
            makerMacroRunning = false;
            toggleSquareG2 = true;
            specimenMade = false;
        }
        if (makerMacro!=null && !makerMacroRunning) makerMacro.update(MovementType.MAGAZINE_FEEDER);
        if (!gamepad2.square) toggleSquareG2 = false;




        /// gamepad 1 triangle activates deposit macro
        if (gamepad1.triangle && !toggleTriangleG1 && specimenMade && !depositMacroRunning) {
            initDepositMacro();
            depositMacro.start();
            toggleTriangleG1 = true;
            depositMacroRunning = true;
            deposited = false;
        }
        // run synchronizer
        if (depositMacroRunning) {
            if (!depositMacro.update()) {
                depositMacroRunning = false;
                deposited = true;
                depositReadyToRelease = true;
                specimenMade = false;
                robot.linearSlides.stopLifts();
            }
        }
        // second press cancels it
        if (gamepad1.triangle && !toggleTriangleG1 && depositMacroRunning) {
            depositMacro.stop();
            depositMacroRunning = false;
            toggleTriangleG1 = true;
            deposited = false;
        }
        if (!gamepad1.triangle) toggleTriangleG1 = false;


        /// gamepad 1 cross can decide when to release specimen
        if (depositReadyToRelease && gamepad1.cross) {
            robot.verticalDeposit.release();
            depositReadyToRelease = false;
        }
    }


    /**
     * Sets the gamepad led color to the cv sample color
     */
    private void handleGamepadColors() {
        // choose primary color
        if (gamepad1.dpad_up) {
            SampleOrientationProcessor.colorType = SampleOrientationProcessor.SampleColor.RED;
        }
        // choose yellow
        if (gamepad1.dpad_down) {
            SampleOrientationProcessor.colorType = SampleOrientationProcessor.SampleColor.YELLOW;
        }

        // visual telemetry for if sample detected
        if (gamepad1HasDriveControl && robot.overheadSampleData.isFilterFull()) {
            switch (SampleOrientationProcessor.colorType) {
                case YELLOW:
                    gamepad2.setLedColor(1, 1, 0, 1000);
                    break;
                case BLUE:
                    gamepad2.setLedColor(0, 0, 1, 1000);
                    break;
                case RED:
                    gamepad2.setLedColor(1, 0, 0, 1000);
                    break;
                default:
                    break;
            }
        }
        // nothing detected = gray gmp2, white gmp1
        else if (gamepad1HasDriveControl) {
            gamepad1.setLedColor(1, 1, 1, 1000);
            gamepad2.setLedColor(0.025, 0.025, 0.025, 1000);
        }
        // gmp2 is white in mag reload mode, gmp1 is gray
        else {
            gamepad1.setLedColor(0.025, 0.025, 0.025, 1000);
            gamepad2.setLedColor(1, 1, 1, 1000);
        }
    }


    private void initialize() {
        // init subsystems
        this.robot = new AutonomousRobot(
                hardwareMap,
                new Pose2d(48.75, -72+18,new Rotation2d(Math.PI/2)),
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
        OverheadCameraSubsystem.CLAW_OFFSET[0] = -2.6;
        SampleDataBufferFilter.FILTER_ERROR_TOLERANCE = 0.15;
        SampleOrientationProcessor.colorType = SampleOrientationProcessor.SampleColor.RED;

        robot.verticalDeposit.setArmPosition(0.5);
        robot.verticalDeposit.setClawPosition(VClawConstants.RELEASE_POSITION);
        robot.horizontalIntake.setArmPosition(0.9);
        robot.horizontalIntake.setWristAngle(0);
        robot.horizontalIntake.setClawPosition(HClawConstants.GRAB_POSITION);

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


    private void initPickupMacro(ExtendoState extendoVelocity) {
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


        // Create Plans
        TranslationPlan translationPlan = new TranslationPlan(drive, localization,
                translation
        );
        RotationPlan rotationPlan = new RotationPlan(drive, localization,
                rotation
        );
        ExtendoPlan extendo_plan = new ExtendoPlan(linearSlides,
                extendoOut
        );
        HWristPlan h_wrist_plan = new HWristPlan(horizontalIntake,
                h_wrist_align
        );
        HArmPlan h_arm_plan = new HArmPlan(horizontalIntake,
                h_arm_down,
                h_arm_up
        );
        HClawPlan h_claw_plan = new HClawPlan(horizontalIntake,
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
                new ExtendoState(linearSlides.getExtendoPosition()),
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
            advanceFeeder = new LinearMFeeder(holdLiftDown.getEndTime() + feederDelayTime,
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
            advanceFeeder = new LinearMFeeder(holdLiftDown.getEndTime() + feederDelayTime,
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
        ExtendoPlan extendo_plan = new ExtendoPlan(linearSlides,
                extendoIn,
                extendoToTransfer,
                pullSampleIn
        );
        HWristPlan h_wrist_plan = new HWristPlan(horizontalIntake,
                h_wrist_flat,
                h_wrist_reset
        );
        HArmPlan h_arm_plan = new HArmPlan(horizontalIntake,
                hArmUpTransfer,
                hArmDown
        );
        HClawPlan h_claw_plan = new HClawPlan(horizontalIntake,
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




    private void initDepositMacro() {

        // Score specimen
        LinearVArm vArmToPreDepositCycle = new LinearVArm(0,
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
