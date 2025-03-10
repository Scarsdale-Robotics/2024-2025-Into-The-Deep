package org.firstinspires.ftc.teamcode.opmodes.autons;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.opmodes.algorithms.SampleDataBufferFilter;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LocalizationSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.AutonomousRobot;
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
import org.firstinspires.ftc.teamcode.synchropather.systems.hClaw.movements.ReleaseHClaw;
import org.firstinspires.ftc.teamcode.synchropather.systems.hWrist.HWristPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.hWrist.movements.MoveHWrist;
import org.firstinspires.ftc.teamcode.synchropather.systems.rotation.RotationConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.rotation.RotationPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.rotation.RotationState;
import org.firstinspires.ftc.teamcode.synchropather.systems.rotation.movements.LinearRotation;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.TranslationConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.TranslationPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.TranslationState;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.movements.CRSplineTranslation;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.movements.LinearTranslation;

@Config
@Disabled
@Autonomous(name="Better sample cycle auto (trust)")
public class BetterSampleCycleAuto extends LinearOpMode {

    private Synchronizer search, pickup, score;

    private AutonomousRobot robot;

    private HorizontalIntakeSubsystem horizontalIntake;
    private OverheadCameraSubsystem overheadCamera;
    private LinearSlidesSubsystem linearSlides;
    private LocalizationSubsystem localization;
    private DriveSubsystem drive;

    private SampleDataBufferFilter sampleData;

    public static double armDownPosition = 1.04;

    public static double intakeDelay = 0.35;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize(new Pose2d(-39, -63, new Rotation2d(Math.toRadians(0))));
        initScoreAction(new ExtendoState(0), true);
        telemetry.addData("score.getDuration()", score.getDuration());
        telemetry.update();

        waitForStart();


        // Preload
        score.start();
        while (opModeIsActive() && score.update()) robot.update();

        // Search spike mark 1
        initSearchAction(new TranslationState(-48, -24));
        search.start();
        ExtendoState extendoVelocity = new ExtendoState(0);
        while (opModeIsActive() && (search.update() || !sampleData.isFilterFull())) {
            robot.update();
            if (!sampleData.isFilterFull()) {
                sampleData.updateFilterData(overheadCamera.getSamplePositions(), overheadCamera.getSampleAngles(), overheadCamera.getClosestSample()); // Can return null
            }
            extendoVelocity = (ExtendoState) search.getVelocity(MovementType.EXTENDO);
            if (sampleData.isFilterFull()) break;
        }

        // Pickup spike mark 1
        initPickupMotion(extendoVelocity);
        pickup.start();
        while (opModeIsActive() && pickup.update()) {
            robot.update();
            extendoVelocity = (ExtendoState) pickup.getVelocity(MovementType.EXTENDO);
        }

        // Score spike mark 1
        initScoreAction(extendoVelocity, false);
        score.start();
        while (opModeIsActive() && score.update()) robot.update();



        // Search spike mark 2
        initSearchAction(new TranslationState(-58, -24));
        sampleData.clearFilterData();
        search.start();
        while (opModeIsActive() && (search.update() || !sampleData.isFilterFull())) {
            robot.update();
            if (!sampleData.isFilterFull()) {
                sampleData.updateFilterData(overheadCamera.getSamplePositions(), overheadCamera.getSampleAngles(), overheadCamera.getClosestSample()); // Can return null
            }
            extendoVelocity = (ExtendoState) search.getVelocity(MovementType.EXTENDO);
            if (sampleData.isFilterFull()) break;
        }

        // Pickup spike mark 2
        initPickupMotion(extendoVelocity);
        pickup.start();
        while (opModeIsActive() && pickup.update()) {
            robot.update();
            extendoVelocity = (ExtendoState) pickup.getVelocity(MovementType.EXTENDO);
        }

        // Score spike mark 2
        initScoreAction(extendoVelocity, false);
        score.start();
        while (opModeIsActive() && score.update()) robot.update();


        // Cycle sub
        for (int i = 0; opModeIsActive() && i < 3; i++) {

            // Search sub
            initSearchAction(new TranslationState(0, 0));
            sampleData.clearFilterData();
            search.start();
            while (opModeIsActive() && (search.update() || !sampleData.isFilterFull())) {
                robot.update();
                if (!sampleData.isFilterFull()) {
                    sampleData.updateFilterData(overheadCamera.getSamplePositions(), overheadCamera.getSampleAngles(), overheadCamera.getClosestSample()); // Can return null
                }
                extendoVelocity = (ExtendoState) search.getVelocity(MovementType.EXTENDO);
                if (sampleData.isFilterFull()) break;
            }

            // Pickup sub
            initPickupMotion(extendoVelocity);
            pickup.start();
            while (opModeIsActive() && pickup.update()) {
                robot.update();
                extendoVelocity = (ExtendoState) pickup.getVelocity(MovementType.EXTENDO);
            }

            // Score
            initScoreAction(extendoVelocity, false);
            score.start();
            while (opModeIsActive() && score.update()) robot.update();
        }


    }


    private void initialize(Pose2d initialPose) {
        // init subsystems
        this.robot = new AutonomousRobot(
                hardwareMap,
                initialPose,
                AutonomousRobot.TeamColor.BLUE,
                this,
                SampleDataBufferFilter.SampleTargetingMethod.ROTATION
        );
        this.telemetry = robot.telemetry;
        this.horizontalIntake = robot.horizontalIntake;
        this.overheadCamera = robot.overheadCamera;
        this.linearSlides = robot.linearSlides;
        this.localization = robot.localization;
        this.drive = robot.drive;
        OverheadCameraSubsystem.CLAW_OFFSET[0] = -2;
        ExtendoConstants.MAX_MOTOR_VELOCITY = 80;

        this.sampleData = new SampleDataBufferFilter(
                linearSlides,
                localization,
                0.04375,
                1,
                SampleDataBufferFilter.SampleTargetingMethod.ROTATION
        );
        robot.setOverheadSampleDataBufferFilter(this.sampleData);
        SampleDataBufferFilter.FILTER_ERROR_TOLERANCE = 0.5;

        // init servos
        horizontalIntake.setClawPosition(HClawConstants.GRAB_POSITION);
        horizontalIntake.setWristAngle(Math.PI/2);
        horizontalIntake.setArmPosition(0.25);
    }

    private void initPickupMotion(ExtendoState extendoVelocity) {
        RotationConstants.MAX_ANGULAR_VELOCITY = 0.65*3.8;
        RotationConstants.MAX_ANGULAR_ACCELERATION = 0.65*8;

        ExtendoConstants.MAX_PATHING_VELOCITY = 40;
        ExtendoConstants.MAX_ACCELERATION = 40;
        ExtendoPlan.kA = 0;

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
        if (samplePose==null) {
            pickup = new Synchronizer();
            return;
        }
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
        GrabHClaw h_claw_grab = new GrabHClaw(h_arm_down.getEndTime()-0.15, 0.35);

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
                h_arm_down
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



    private void initScoreAction(ExtendoState extendoVelocity, boolean armAtScoringPosition) {
        TranslationConstants.MAX_VELOCITY = 40d;
        TranslationConstants.MAX_ACCELERATION = 54d;

        RotationConstants.MAX_ANGULAR_VELOCITY = 0.5*3.6;
        RotationConstants.MAX_ANGULAR_ACCELERATION = 8;

        ExtendoConstants.MAX_PATHING_VELOCITY = 40;
        ExtendoConstants.MAX_ACCELERATION = 80;

        ExtendoPlan.kA = 0.2;

        // Extract current state
        Pose2d botPose = localization.getPose();
        TranslationState currentTranslation = new TranslationState(botPose);
        RotationState currentRotation = new RotationState(botPose);
        ExtendoState currentExtendo = new ExtendoState(linearSlides.getExtendoPosition());

        // Unpack bot pose
        double x_bot = currentTranslation.getX();
        double y_bot = currentTranslation.getY();

        // Plans
        TranslationPlan translationPlan;
        RotationPlan rotationPlan;
        if (y_bot > -24) {
            CRSplineTranslation spline = new CRSplineTranslation(0,
                    currentTranslation,
                    new TranslationState(-48, -24),
                    new TranslationState(-60, -48)
            );
            LinearRotation rotation = new LinearRotation(0,
                    currentRotation,
                    new RotationState(Math.atan2(15,3))
            );
            translationPlan = new TranslationPlan(drive, localization,
                    spline
            );
            rotationPlan = new RotationPlan(drive, localization,
                    rotation
            );
        } else {
            double targetHeading = Math.atan2(y_bot+63, x_bot+63);
            LinearTranslation line = new LinearTranslation(0,
                    currentTranslation,
                    new TranslationState(
                            -63+15*Math.cos(targetHeading),
                            -63+15*Math.sin(targetHeading)
                    )
            );
            LinearRotation rotation = new LinearRotation(0,
                    currentRotation,
                    new RotationState(targetHeading)
            );
            translationPlan = new TranslationPlan(drive, localization,
                    line
            );
            rotationPlan = new RotationPlan(drive, localization,
                    rotation
            );
        }

        // Extendo
        DynamicLinearExtendo retractExtendo = new DynamicLinearExtendo(0,
                currentExtendo,
                new ExtendoState(LinearSlidesSubsystem.extendoOffset),
                extendoVelocity
        );
        ExtendoPlan extendoPlan = new ExtendoPlan(linearSlides,
                retractExtendo
        );

        // Bring arm back
        LinearHArm linearHArm = new LinearHArm(0,
                new HArmState(armAtScoringPosition ? 0.25 : 0.9),
                new HArmState(0.25) // Back of robot
        );
        HArmPlan hArmPlan = new HArmPlan(horizontalIntake,
                linearHArm
        );

        // Make wrist flat
        MoveHWrist flatWrist = new MoveHWrist(0, Math.PI/2);
        HWristPlan hWristPlan = new HWristPlan(horizontalIntake, flatWrist);

        // Release claw
        double releaseTime = max(extendoPlan.getDuration(), translationPlan.getDuration(), rotationPlan.getDuration(), hArmPlan.getDuration());
        ReleaseHClaw releaseHClaw = new ReleaseHClaw(releaseTime, true);
        HClawPlan hClawPlan = new HClawPlan(horizontalIntake, releaseHClaw);

        score =  new Synchronizer(
                translationPlan,
                rotationPlan,
                extendoPlan,
                hArmPlan,
                hWristPlan,
                hClawPlan
        );
    }


    private void initSearchAction(TranslationState targetTranslation) {
        TranslationConstants.MAX_VELOCITY = 40d;
        TranslationConstants.MAX_ACCELERATION = 54d;

        ExtendoConstants.MAX_PATHING_VELOCITY = 40;
        ExtendoConstants.MAX_ACCELERATION = 40;
        ExtendoPlan.kA = 0;

        if (targetTranslation.getX()==-48) {
            RotationConstants.MAX_ANGULAR_VELOCITY = 3.6;
        } else {
            RotationConstants.MAX_ANGULAR_VELOCITY = 0.5*3.6;
        }
        RotationConstants.MAX_ANGULAR_ACCELERATION = 8;

        // Extendo
        double previousMaxVelocity = ExtendoConstants.MAX_PATHING_VELOCITY;

        // Extract current state
        Pose2d botPose = localization.getPose();
        TranslationState currentTranslation = new TranslationState(botPose);
        RotationState currentRotation = new RotationState(botPose);
        ExtendoState currentExtendo = new ExtendoState(linearSlides.getExtendoPosition());


        // Plans
        TranslationPlan translationPlan;
        RotationPlan rotationPlan;
        ExtendoPlan extendoPlan;

        // Case: search submersible
        if (targetTranslation.getY()>-24) {
            CRSplineTranslation spline = new CRSplineTranslation(0,
                    currentTranslation,
                    new TranslationState(-48,-24),
                    new TranslationState(-18,-12)
            );
            LinearRotation rotation = new LinearRotation(0,
                    currentRotation,
                    new RotationState(0)
            );
            translationPlan = new TranslationPlan(drive, localization,
                    spline
            );
            rotationPlan = new RotationPlan(drive, localization,
                    rotation
            );

            ExtendoConstants.MAX_PATHING_VELOCITY = 0.5*previousMaxVelocity;
            LinearExtendo extend1 = new LinearExtendo(max(translationPlan.getDuration(), rotationPlan.getDuration())-0.25,
                    new ExtendoState(LinearSlidesSubsystem.extendoOffset),
                    new ExtendoState(ExtendoConstants.MAX_EXTENSION)
            );
            LinearExtendo retract1 = new LinearExtendo(extend1.getEndTime(),
                    new ExtendoState(ExtendoConstants.MAX_EXTENSION),
                    new ExtendoState(LinearSlidesSubsystem.extendoOffset)
            );
            LinearExtendo extend2 = new LinearExtendo(retract1.getEndTime(),
                    new ExtendoState(LinearSlidesSubsystem.extendoOffset),
                    new ExtendoState(ExtendoConstants.MAX_EXTENSION)
            );
            LinearExtendo retract2 = new LinearExtendo(extend2.getEndTime(),
                    new ExtendoState(ExtendoConstants.MAX_EXTENSION),
                    new ExtendoState(LinearSlidesSubsystem.extendoOffset)
            );
            extendoPlan = new ExtendoPlan(linearSlides,
                    extend1,
                    retract1,
                    extend2,
                    retract2
            );
        }
        // Case: spike mark
        else {
            LinearTranslation line = new LinearTranslation(0,
                    currentTranslation,
                    new TranslationState(targetTranslation.getX(), -48)
            );
            LinearRotation rotation = new LinearRotation(0,
                    currentRotation,
                    new RotationState(Math.toRadians(90))
            );
            translationPlan = new TranslationPlan(drive, localization,
                    line
            );
            rotationPlan = new RotationPlan(drive, localization,
                    rotation
            );

            // Extendo
            ExtendoConstants.MAX_PATHING_VELOCITY = 0.5*previousMaxVelocity;
            LinearExtendo extend = new LinearExtendo(max(translationPlan.getDuration(), rotationPlan.getDuration())-0.5,
                    currentExtendo,
                    new ExtendoState(13.5)
            );
            extendoPlan = new ExtendoPlan(linearSlides,
                    extend
            );
        }
        ExtendoConstants.MAX_PATHING_VELOCITY = previousMaxVelocity;

        // Lower arm
        LinearHArm lowerArm = new LinearHArm(0,
                new HArmState(0.25),
                new HArmState(0.9)
        );
        HArmPlan hArmPlan = new HArmPlan(horizontalIntake, lowerArm);

        // Reset wrist
        MoveHWrist resetWrist = new MoveHWrist(0, 0);
        HWristPlan hWristPlan = new HWristPlan(horizontalIntake, resetWrist);

        search =  new Synchronizer(
                translationPlan,
                rotationPlan,
                extendoPlan,
                hArmPlan,
                hWristPlan
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

    /**
     * Clips the input x between a given lower and upper bound.
     * @param x
     * @param lower
     * @param upper
     * @return the clipped value of x.
     */
    private static double bound(double x, double lower, double upper) {
        return Math.max(lower, Math.min(upper, x));
    }

    private static double max(double... values) {
        if (values.length==0) return -1;
        double maxValue = values[0];
        for (int i = 1; i < values.length; i++) {
            maxValue = Math.max(maxValue, values[i]);
        }
        return maxValue;
    }

}
