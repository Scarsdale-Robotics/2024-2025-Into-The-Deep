package org.firstinspires.ftc.teamcode.synchropather.macros;

import com.arcrobotics.ftclib.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.opmodes.algorithms.SampleDataBufferFilter.SampleTargetingMethod;
import org.firstinspires.ftc.teamcode.synchropather.AutonomousRobot;
import org.firstinspires.ftc.teamcode.synchropather.subsystemclasses.HorizontalIntakeSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.subsystemclasses.LinearSlidesSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.subsystemclasses.OverheadCameraSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.Synchronizer;
import org.firstinspires.ftc.teamcode.synchropather.systems.extendo.ExtendoConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.extendo.ExtendoPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.extendo.ExtendoState;
import org.firstinspires.ftc.teamcode.synchropather.systems.extendo.movements.DynamicLinearExtendo;
import org.firstinspires.ftc.teamcode.synchropather.systems.extendo.movements.LinearExtendo;
import org.firstinspires.ftc.teamcode.synchropather.systems.hArm.HArmPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.hArm.HArmState;
import org.firstinspires.ftc.teamcode.synchropather.systems.hArm.movements.LinearHArm;
import org.firstinspires.ftc.teamcode.synchropather.systems.hClaw.HClawPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.hClaw.movements.ReleaseHClaw;
import org.firstinspires.ftc.teamcode.synchropather.systems.hWrist.HWristPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.hWrist.movements.MoveHWrist;
import org.firstinspires.ftc.teamcode.synchropather.systems.rotation.RotationPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.rotation.RotationState;
import org.firstinspires.ftc.teamcode.synchropather.systems.rotation.movements.LinearRotation;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.TranslationPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.TranslationState;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.movements.LinearTranslation;

public class EducatedSearchMacro extends Synchronizer {

    public EducatedSearchMacro(double[] samplePosition, AutonomousRobot robot, double speedFactor) {
        TranslationPlan translationPlan;
        RotationPlan rotationPlan;
        ExtendoPlan extendoPlan;
        if (robot.sampleTargetingMethod==SampleTargetingMethod.TRANSLATION) {
            // Unpack bot pose
            Pose2d botPose = robot.localization.getPose();
            double x_bot = botPose.getX();
            double y_bot = botPose.getY();
            double heading_bot = botPose.getHeading();

            // Unpack sample pose
            double x_sample = samplePosition[0];
            double y_sample = samplePosition[1];

            // Get extendo state
            double x_extendo = robot.linearSlides.getExtendoPosition();
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
                 Math.max(0, (y_sample-y_bot) - x_extendo_min + OverheadCameraSubsystem.CLAW_OFFSET[0])
            );

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

            // Extend and retract
            double previousMaxVelocity = ExtendoConstants.MAX_PATHING_VELOCITY;
            ExtendoConstants.MAX_PATHING_VELOCITY = speedFactor*previousMaxVelocity;
            LinearExtendo extendoOut = new LinearExtendo(0,
                    extendoPosition,
                    extendoTarget
            );
            LinearExtendo extendoIn = new LinearExtendo(extendoOut.getEndTime(),
                    extendoTarget,
                    new ExtendoState(LinearSlidesSubsystem.extendoOffset)
            );
            ExtendoConstants.MAX_PATHING_VELOCITY = previousMaxVelocity;

            // Plans
            translationPlan = new TranslationPlan(robot.drive, robot.localization,
                    translation
            );
            rotationPlan = new RotationPlan(robot.drive, robot.localization,
                    rotation
            );
            extendoPlan = new ExtendoPlan(robot.linearSlides,
                    extendoOut,
                    extendoIn
            );
        }
        // Case: targeting method is ROTATION
        else {
            // Unpack bot pose
            Pose2d botPose = robot.localization.getPose();
            double x_bot = botPose.getX();
            double y_bot = botPose.getY();
            double heading_bot = botPose.getHeading();

            // Get extendo state
            double x_extendo = robot.linearSlides.getExtendoPosition();
            ExtendoState extendoPosition = new ExtendoState(x_extendo);

            // Unpack sample pose
            double x_sample = samplePosition[0];
            double y_sample = samplePosition[1];

            // Convert global sample pose to robot frame
            double dx = x_sample - x_bot;
            double dy = y_sample - y_bot;
            double sin = Math.sin(-heading_bot);
            double cos = Math.cos(-heading_bot);
            double x_sample_bot = dx*cos - dy*sin;
            double y_sample_bot = dx*sin + dy*cos;

            // Calculate heading difference given horizontal claw offset
            double r_sample_bot_norm = Math.hypot(x_sample_bot, y_sample_bot);
            double theta_sample_tangent = Math.atan2(y_sample_bot, x_sample_bot) - Math.acos(OverheadCameraSubsystem.CAMERA_OFFSET[1]/r_sample_bot_norm);
            double delta_heading = theta_sample_tangent + Math.PI/2;

            // Extendo prep calculations
            double rcos = OverheadCameraSubsystem.CAMERA_OFFSET[1]*Math.cos(theta_sample_tangent);
            double rsin = OverheadCameraSubsystem.CAMERA_OFFSET[1]*Math.sin(theta_sample_tangent);
            double d_sample_bot = Math.hypot(rcos-x_sample_bot, rsin-y_sample_bot);

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

            // Extend and retract
            LinearExtendo extendoOut = new LinearExtendo(0,
                    extendoPosition,
                    extendoTarget
            );
            LinearExtendo extendoIn = new LinearExtendo(extendoOut.getEndTime(),
                    extendoTarget,
                    new ExtendoState(0)
            );

            // Plans
            translationPlan = new TranslationPlan(robot.drive, robot.localization,
                    translation
            );
            rotationPlan = new RotationPlan(robot.drive, robot.localization,
                    rotation
            );
            extendoPlan = new ExtendoPlan(robot.linearSlides,
                    extendoOut,
                    extendoIn
            );
        }


        // Keep other subsystems still
        MoveHWrist h_wrist_reset = new MoveHWrist(0, 0);
        LinearHArm h_arm_up = new LinearHArm(0,
                new HArmState(0.9),
                new HArmState(0.9)
        );

        // Create Plans
        HWristPlan h_wrist_plan = new HWristPlan(robot.horizontalIntake,
                h_wrist_reset
        );
        HArmPlan h_arm_plan = new HArmPlan(robot.horizontalIntake,
                h_arm_up
        );

        setPlans(
                translationPlan,
                rotationPlan,
                extendoPlan,
                h_wrist_plan,
                h_arm_plan
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
