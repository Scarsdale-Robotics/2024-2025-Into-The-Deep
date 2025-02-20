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
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LocalizationSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.AutonomousRobot;
import org.firstinspires.ftc.teamcode.synchropather.macros.ExtendoRetractMacro;
import org.firstinspires.ftc.teamcode.synchropather.macros.SearchMacro;
import org.firstinspires.ftc.teamcode.synchropather.subsystemclasses.HorizontalIntakeSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.subsystemclasses.LinearSlidesSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.subsystemclasses.OverheadCameraSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.systems.MovementType;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.Synchronizer;
import org.firstinspires.ftc.teamcode.synchropather.systems.extendo.ExtendoConstants;
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
import org.firstinspires.ftc.teamcode.synchropather.systems.hWrist.HWristPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.hWrist.movements.MoveHWrist;
import org.firstinspires.ftc.teamcode.synchropather.systems.rotation.RotationPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.rotation.RotationState;
import org.firstinspires.ftc.teamcode.synchropather.systems.rotation.movements.LinearRotation;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.TranslationPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.TranslationState;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.movements.LinearTranslation;

@Config
@TeleOp(name="Drive Rotation Macro Test", group = "Calibration")
public class DriveRotationMacroTest extends LinearOpMode {

    private Synchronizer search, pickup;
    private Synchronizer extendoRetract;

    private AutonomousRobot robot;

    private HorizontalIntakeSubsystem horizontalIntake;
    private OverheadCameraSubsystem overheadCamera;
    private LinearSlidesSubsystem linearSlides;
    private LocalizationSubsystem localization;
    private DriveSubsystem drive;

    private SampleDataBufferFilter sampleData;

    public static double armDownPosition = 1.025;

    /**
     * Between 0 and 1 (please).
     */
    public static double searchSpeedFactor = 0.5;

    public static double driveSpeed = 1;

    public static double intakeDelay = 0.25;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();

        boolean toggleTriangle = false;
        boolean sampleMacroRunning = false;
        boolean clawGrabbed = false;
        ExtendoState extendoVelocity = null;
        while (opModeIsActive()) {
            robot.update();
            handleGamepadColor();
            boolean driverControlling = controlDrive(sampleMacroRunning);


            //// Triangle button
            // Case: Init search macro
            if (gamepad1.triangle && !toggleTriangle && !sampleMacroRunning && !clawGrabbed) {
                drive.stopController();
                // init search
                search = new SearchMacro(
                        16,
                        searchSpeedFactor,
                        linearSlides,
                        horizontalIntake
                );
                search.start();
                sampleData.clearFilterData();
                sampleMacroRunning = true;
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
        double forward = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
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
        if (gamepad1.dpad_up) {
            SampleOrientationProcessor.colorType = SampleOrientationProcessor.SampleColor.YELLOW;
        }
        if (gamepad1.dpad_left) {
            SampleOrientationProcessor.colorType = SampleOrientationProcessor.SampleColor.BLUE;
        }
        if (gamepad1.dpad_right) {
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
                new Pose2d(1,1,new Rotation2d(0)),
                AutonomousRobot.TeamColor.BLUE,
                this
        );
        this.telemetry = robot.telemetry;
        this.horizontalIntake = robot.horizontalIntake;
        this.overheadCamera = robot.overheadCamera;
        this.linearSlides = robot.linearSlides;
        this.localization = robot.localization;
        this.drive = robot.drive;
        this.sampleData = robot.sampleData;
        OverheadCameraSubsystem.CLAW_OFFSET[0] = -2;

        // init extendo retract macro
        extendoRetract = new ExtendoRetractMacro(linearSlides);
        extendoRetract.start();
    }


    private void initPickupMotion(ExtendoState extendoVelocity) {
        // Unpack bot pose
        Pose2d botPose = localization.getPose();
        double x_bot = botPose.getX();
        double y_bot = botPose.getY();
        double heading_bot = botPose.getHeading();

        // Get extendo state
        double x_extendo = linearSlides.getExtendoPosition();
        ExtendoState extendoPosition = new ExtendoState(x_extendo);

        // Unpack sample pose
        Pose2d samplePose = sampleData.getFilteredSamplePosition(telemetry);
        double x_sample = samplePose.getX();
        double y_sample = samplePose.getY();
        double theta_sample = samplePose.getHeading();

        // Convert global sample pose to robot frame
        double dx = x_sample - x_bot;
        double dy = y_sample - y_bot;
        double sin = Math.sin(-heading_bot);
        double cos = Math.cos(-heading_bot);
        double x_sample_bot = dx*cos - dy*sin;
        double y_sample_bot = dx*sin + dy*cos;
        double theta_sample_bot = theta_sample - heading_bot + Math.PI/2.0;

        telemetry.addData("[DEBUG] x_sample_bot", x_sample_bot);
        telemetry.addData("[DEBUG] y_sample_bot", y_sample_bot);
        telemetry.addData("[DEBUG] theta_sample_bot", theta_sample_bot);

        // Calculate heading difference given horizontal claw offset
        double r_sample_bot_norm = Math.hypot(x_sample_bot, y_sample_bot);
        double theta_sample_tangent = Math.atan2(y_sample_bot, x_sample_bot) - Math.acos(OverheadCameraSubsystem.CAMERA_OFFSET[1]/r_sample_bot_norm);
        double delta_heading = theta_sample_tangent + Math.PI/2;

        // Extendo prep calculations
        double rcos = OverheadCameraSubsystem.CAMERA_OFFSET[1]*Math.cos(theta_sample_tangent);
        double rsin = OverheadCameraSubsystem.CAMERA_OFFSET[1]*Math.sin(theta_sample_tangent);
        double d_sample_bot = Math.hypot(rcos-x_sample_bot, rsin-y_sample_bot);

        telemetry.addData("[DEBUG] r_sample_bot_norm", r_sample_bot_norm);
        telemetry.addData("[DEBUG] theta_sample_tangent", theta_sample_tangent);
        telemetry.addData("[DEBUG] delta_heading", delta_heading);

        // Get subsystem setpoints
        TranslationState translationTarget = new TranslationState(
                botPose
        );
        RotationState rotationTarget = new RotationState(
                botPose.getHeading()+delta_heading
        );
        ExtendoState extendoTarget = new ExtendoState(
                d_sample_bot - (OverheadCameraSubsystem.CAMERA_OFFSET[0] + OverheadCameraSubsystem.CLAW_OFFSET[0])
        );
        double hWristTarget = normalizeAngle(theta_sample_bot - delta_heading);


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

        double previousMaxVelocity = ExtendoConstants.MAX_PATHING_VELOCITY;
        ExtendoConstants.MAX_PATHING_VELOCITY = previousMaxVelocity / 3;
        DynamicLinearExtendo extendoOut = new DynamicLinearExtendo(0,
                extendoPosition,
                extendoTarget,
                extendoVelocity
        );
        ExtendoConstants.MAX_PATHING_VELOCITY = previousMaxVelocity;

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
                new ExtendoState(0)
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
