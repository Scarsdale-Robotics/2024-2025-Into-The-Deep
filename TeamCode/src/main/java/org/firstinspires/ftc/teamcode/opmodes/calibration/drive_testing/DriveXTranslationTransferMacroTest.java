package org.firstinspires.ftc.teamcode.opmodes.calibration.drive_testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.cvprocessors.SampleOrientationProcessor;
import org.firstinspires.ftc.teamcode.opmodes.algorithms.SampleDataBufferFilter;
import org.firstinspires.ftc.teamcode.opmodes.calibration.Drawing;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LocalizationSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.AutonomousRobot;
import org.firstinspires.ftc.teamcode.synchropather.macros.EducatedSearchMacro;
import org.firstinspires.ftc.teamcode.synchropather.macros.ExtendoRetractMacro;
import org.firstinspires.ftc.teamcode.synchropather.subsystemclasses.HorizontalIntakeSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.subsystemclasses.LinearSlidesSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.subsystemclasses.OverheadCameraSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.systems.MovementType;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.Synchronizer;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.TimeSpan;
import org.firstinspires.ftc.teamcode.synchropather.systems.extendo.ExtendoPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.extendo.ExtendoState;
import org.firstinspires.ftc.teamcode.synchropather.systems.extendo.movements.DynamicLinearExtendo;
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
import org.firstinspires.ftc.teamcode.synchropather.systems.lift.LiftConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.lift.LiftPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.lift.LiftState;
import org.firstinspires.ftc.teamcode.synchropather.systems.lift.movements.LinearLift;
import org.firstinspires.ftc.teamcode.synchropather.systems.limelight.LimelightPipeline;
import org.firstinspires.ftc.teamcode.synchropather.systems.limelight.LimelightPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.limelight.movements.EnableLimelight;
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
import org.firstinspires.ftc.teamcode.synchropather.systems.vClaw.movements.GrabVClaw;

import java.util.List;

@Config
@TeleOp(name="Drive X Translation Transfer Macro Test", group = "Calibration")
public class DriveXTranslationTransferMacroTest extends LinearOpMode {

    private Synchronizer search, pickup;
    private Synchronizer extendoRetract;

    private AutonomousRobot robot;

    private HorizontalIntakeSubsystem horizontalIntake;
    private OverheadCameraSubsystem overheadCamera;
    private LinearSlidesSubsystem linearSlides;
    private LocalizationSubsystem localization;
    private DriveSubsystem drive;

    private SampleDataBufferFilter sampleData;

    public static double armDownPosition = 1.04;

    public static double driveSpeed = 1;

    public static double intakeDelay = 0.25;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();


        EnableLimelight enableLimelight = new EnableLimelight(new TimeSpan(0,10), LimelightPipeline.SAMPLE_DETECTOR);
        LimelightPlan limelightPlan = new LimelightPlan(robot.limelightSubsystem, enableLimelight);
        Synchronizer limelightAction = new Synchronizer(limelightPlan);
        limelightAction.start();

        boolean toggleTriangle = false;
        boolean sampleMacroRunning = false;
        boolean clawGrabbed = false;
        ExtendoState extendoVelocity = null;
        while (opModeIsActive()) {
            limelightAction.update();
            robot.update();
            boolean driverControlling = controlDrive(sampleMacroRunning);


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
                            robot
                    );
                    search.start();
                    sampleData.clearFilterData();
                    sampleMacroRunning = true;
                }
            }
            // Case: Drop off sample
            else if (gamepad1.triangle && !toggleTriangle && !sampleMacroRunning && clawGrabbed) {
                horizontalIntake.setClawPosition(HClawConstants.RELEASE_POSITION);
                clawGrabbed = false;
            }
            // Case: Cancel search macro
            else if (gamepad1.triangle && !toggleTriangle && sampleMacroRunning) {
                sampleMacroRunning = false;
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
                    // Stop macro if driver took over (or macro ended)
                    if (driverControlling || (pickup.getIsRunning() && !pickup.update())) {
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


            //// Draw detected sample position
            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), localization.getPose());
            Pose2d samplePosition = sampleData.getFilteredSamplePosition(telemetry);
            if (samplePosition!=null) {
                telemetry.addData("samplePosition", samplePosition.toString());
                Drawing.drawTargetPose(packet.fieldOverlay(), samplePosition);
            }
            FtcDashboard.getInstance().sendTelemetryPacket(packet);

        }
    }


    /**
     * Field centric drive (based on driver POV)
     * @return whether or not the driver is giving joystick inputs
     */
    private boolean controlDrive(boolean macroRunning) {
        double forward = gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;
        boolean driving = !(forward==0 && strafe==0 && turn==0);

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
        this.sampleData = robot.overheadSampleData;
        OverheadCameraSubsystem.CLAW_OFFSET[0] = -2.5;

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

        // Get extendo state
        double x_extendo = linearSlides.getExtendoPosition();
        ExtendoState extendoPosition = new ExtendoState(x_extendo);
        double x_extendo_min = OverheadCameraSubsystem.CAMERA_OFFSET[0] + OverheadCameraSubsystem.CLAW_OFFSET[0];

        // Get subsystem setpoints
        TranslationState translationTarget = new TranslationState(
                x_sample,
                y_bot + Math.min(0, (y_sample-y_bot) - x_extendo_min)
        );
        RotationState rotationTarget = new RotationState(
                heading_bot + normalizeAngle(Math.PI/2-heading_bot)
        );
        ExtendoState extendoTarget = new ExtendoState(
                Math.max(0, (y_sample-y_bot) - x_extendo_min)
        );
        double hWristTarget = normalizeAngle(theta_sample);

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
        LinearHArm h_arm_up = new LinearHArm(h_claw_grab.getEndTime(),
                new HArmState(armDownPosition),
                new HArmState(0.9)
        );
        MoveHWrist h_wrist_reset = new MoveHWrist(h_arm_up.getEndTime(), 0, true);

        // Retract extendo
        LinearExtendo extendoIn = new LinearExtendo(h_wrist_reset.getStartTime(),
                extendoTarget,
                new ExtendoState(4.3)
        );

        LinearLift liftUp = new LinearLift(0,
                new LiftState(robot.linearSlides.getLeftLiftPosition()),
                new LiftState(LiftConstants.transferPosition)
        );

        // Horizontal arm goes up
        LinearHArm hArmUp = new LinearHArm(Math.max(extendoIn.getEndTime(), liftUp.getEndTime()),
                new HArmState(0.9),
                new HArmState(0.48)
        );

        // Vertical arm gets ready
        LinearVArm vArmDown = new LinearVArm(hArmUp.getEndTime(),
                new VArmState(0.5),
                new VArmState(VArmConstants.armLeftTransferPosition)
        );

        // Deposit claw grabs sample
        GrabVClaw grabVClaw = new GrabVClaw(vArmDown.getEndTime() + 0.25);

        // Intake claw releases sample
        ReleaseHClaw releaseHClaw = new ReleaseHClaw(grabVClaw.getEndTime());

        // Deposit arm moves out of the way
        LinearVArm upVArm = new LinearVArm(releaseHClaw.getEndTime(),
                new VArmState(VArmConstants.armLeftTransferPosition),
                new VArmState(VArmConstants.armLeftUpPosition)
        );

        // Intake arm moves back down
        LinearHArm hArmDown = new LinearHArm(upVArm.getEndTime(),
                new HArmState(0.48),
                new HArmState(0.9)
        );

        // Deposit arm prepares to meet clipper
        LinearVArm toClipperVArm = new LinearVArm(hArmDown.getEndTime(),
                new VArmState(VArmConstants.armLeftUpPosition),
                new VArmState(VArmConstants.armLeftTransferPosition)
        );

        // Lift approaches to meet clipper
        LinearLift toClipperLift = new LinearLift(hArmDown.getEndTime(),
                new LiftState(robot.linearSlides.getLeftLiftPosition()),
                new LiftState(LiftConstants.specMakerPosition)
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
                extendoIn
        );
        HWristPlan h_wrist_plan = new HWristPlan(horizontalIntake,
                h_wrist_align,
                h_wrist_reset
        );
        HArmPlan h_arm_plan = new HArmPlan(horizontalIntake,
                h_arm_down,
                h_arm_up,
                hArmUp,
                hArmDown
        );
        HClawPlan h_claw_plan = new HClawPlan(horizontalIntake,
                h_claw_grab,
                releaseHClaw
        );
        LiftPlan liftPlan = new LiftPlan(robot.linearSlides,
                liftUp,
                toClipperLift
        );
        VArmPlan vArmPlan = new VArmPlan(robot.verticalDeposit,
                vArmDown,
                upVArm,
                toClipperVArm
        );
        VClawPlan vClawPlan = new VClawPlan(robot.verticalDeposit,
                grabVClaw
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
                vClawPlan
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
