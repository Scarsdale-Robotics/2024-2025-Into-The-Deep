package org.firstinspires.ftc.teamcode.opmodes.calibration.cv_testing;

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

import java.util.ArrayDeque;
import java.util.ArrayList;

@Config
@TeleOp(name="Sample Position Logger", group = "Calibration")
public class SamplePositionLogger extends LinearOpMode {

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

        double lastRuntime = 0;
        double lastExtendoPower = Double.MIN_VALUE;

        boolean depositReadyToRelease = false;

        boolean intakeMacroRunning = false;
        boolean intookSample = false;
        boolean toggleTriangleG2 = false;

        boolean makerMacroRunning = false;
        boolean specimenMade = false;
        boolean toggleSquareG2 = false;

        boolean depositMacroRunning = false;
        boolean deposited = true;
        boolean toggleCrossG2 = false;

        boolean previousDriverControlling = true;
        while (opModeIsActive()) {
            robot.update();
            updateTPS();


            if (gamepad1.cross) {
                robot.clipbot.setMagazineLoaderPosition(MLoaderConstants.openPosition);
            } else {
                robot.clipbot.setMagazineLoaderPosition(loaderPressurePosition);
            }




            /// Restock option if empty
            if (!inventoryStocked && gamepad1.square) {
                inventoryStocked = true;
                clipInventory = MFeederConstants.RELOAD_CAPACITY;
            }




            // update sample data from overhead camera
            sampleData.updateFilterData(overheadCamera.getSamplePositions(), overheadCamera.getSampleAngles(), overheadCamera.getClosestSample()); // Can return null



            //// Draw detected sample position
            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), localization.getPose());
            Pose2d samplePosition = sampleData.getFilteredSamplePosition(telemetry);
            if (samplePosition!=null) {
                // Unpack bot pose
                Pose2d botPose = localization.getPose();
                double x_bot = botPose.getX();
                double y_bot = botPose.getY();
                double heading_bot = botPose.getHeading();

                // Unpack sample pose;
                double x_sample = samplePosition.getX();
                double y_sample = samplePosition.getY();
                double theta_sample = samplePosition.getHeading();

                // Convert global sample pose to robot frame
                double dx = x_sample - x_bot;
                double dy = y_sample - y_bot;
                double sin = Math.sin(-heading_bot);
                double cos = Math.cos(-heading_bot);
                double x_sample_bot = dx*cos - dy*sin;
                double y_sample_bot = dx*sin + dy*cos;
                double theta_sample_bot = theta_sample - heading_bot + Math.PI/2.0;
                Pose2d processedSamplePose = new Pose2d(x_sample_bot, y_sample_bot, new Rotation2d(theta_sample_bot));

                ArrayList<double[]> sampPosits = overheadCamera.getSamplePositions();
                if (!sampPosits.isEmpty()) {
                    processedSamplePose = new
                            Pose2d(sampPosits.get(0)[0], sampPosits.get(0)[1], new Rotation2d(0));
                }

                telemetry.addData("processedSamplePose", processedSamplePose.toString());
                Drawing.drawTargetPose(packet.fieldOverlay(), processedSamplePose);
            }
            FtcDashboard.getInstance().sendTelemetryPacket(packet);



            telemetry.update();

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
        SampleOrientationProcessor.colorType = SampleOrientationProcessor.SampleColor.RED;

//        robot.verticalDeposit.setArmPosition(0.5);
//        robot.verticalDeposit.setClawPosition(VClawConstants.RELEASE_POSITION);
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
