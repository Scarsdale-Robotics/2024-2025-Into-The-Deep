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
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.Synchronizer;
import org.firstinspires.ftc.teamcode.synchropather.systems.extendo.ExtendoConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.hClaw.HClawConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.lift.LiftPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.lift.LiftState;
import org.firstinspires.ftc.teamcode.synchropather.systems.lift.movements.LinearLift;
import org.firstinspires.ftc.teamcode.synchropather.systems.rotation.RotationPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.rotation.RotationState;
import org.firstinspires.ftc.teamcode.synchropather.systems.rotation.movements.LinearRotation;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.TranslationConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.TranslationPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.TranslationState;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.movements.CRSplineTranslation;

@Config
@Disabled
@Autonomous(name="good auto (sample)")
public class good_auto extends LinearOpMode {

    private Synchronizer auto;

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
        initSynchronizer();

        waitForStart();


        auto.start();
        while (opModeIsActive() && auto.update()) robot.update();
        auto.stop();

    }


    private void initialize(Pose2d initialPose) {
        // init subsystems
        this.robot = new AutonomousRobot(
                hardwareMap,
                initialPose,
                AutonomousRobot.TeamColor.BLUE,
                this,
                SampleDataBufferFilter.SampleTargetingMethod.TRANSLATION
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
        horizontalIntake.setArmPosition(0.85);
    }

    private void initSynchronizer() {
        TranslationConstants.MAX_ACCELERATION = 25;
        TranslationConstants.MAX_VELOCITY = 20;

        CRSplineTranslation toSubmersible = new CRSplineTranslation(0,
                new TranslationState(-39,-63),
                new TranslationState(-48,-12),
                new TranslationState(-24,0)
        );
        TranslationConstants.MAX_ACCELERATION = 50;

        LinearRotation rotationStill = new LinearRotation(0,
                new RotationState(0),
                new RotationState(0)
        );

        LinearLift liftUp = new LinearLift(0,
                new LiftState(0),
                new LiftState(6)
        );




        TranslationPlan translationPlan = new TranslationPlan(robot.drive, robot.localization,
                toSubmersible
        );

        RotationPlan rotationPlan = new RotationPlan(robot.drive, robot.localization,
                rotationStill
        );

        LiftPlan liftPlan = new LiftPlan(robot.linearSlides,
                liftUp
        );

        auto = new Synchronizer(
                translationPlan,
                rotationPlan,
                liftPlan
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
