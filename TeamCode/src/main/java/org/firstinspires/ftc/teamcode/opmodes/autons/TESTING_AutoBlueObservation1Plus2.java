package org.firstinspires.ftc.teamcode.opmodes.autons;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotSystem;
import org.firstinspires.ftc.teamcode.opmodes.calibration.Drawing;
import org.firstinspires.ftc.teamcode.synchropather.DriveConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.Synchronizer;
import org.firstinspires.ftc.teamcode.synchropather.systems.claw.ClawConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.claw.ClawPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.claw.ClawState;
import org.firstinspires.ftc.teamcode.synchropather.systems.claw.movements.LinearClaw;
import org.firstinspires.ftc.teamcode.synchropather.systems.elbow.ElbowConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.elbow.ElbowPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.elbow.ElbowState;
import org.firstinspires.ftc.teamcode.synchropather.systems.elbow.movements.LinearElbow;
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
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.movements.LinearTranslation;

//@Disabled
@Autonomous(name="[TESTING] Auto Blue Observation 1+2", group = "Autons")
public class TESTING_AutoBlueObservation1Plus2 extends LinearOpMode {

    RobotSystem robot;
    Synchronizer synchronizer;

    public static double clawOpen = ClawConstants.OPEN_POSITION;
    public static double clawClosed = ClawConstants.CLOSED_POSITION;

    public static double elbowUp = ElbowConstants.UP_POSITION;
    public static double elbowDown = ElbowConstants.DOWN_POSITION;

    @Override
    public void runOpMode() throws InterruptedException {
        this.robot = new RobotSystem(hardwareMap, new Pose2d(-24, 63.5, new Rotation2d(Math.toRadians(-90))), false, this);
        robot.inDep.setClawPosition(clawClosed);
        robot.inDep.setElbowPosition(elbowUp);
        initSynchronizer();

        waitForStart();

        synchronizer.start();
        while (opModeIsActive() && synchronizer.update()) {
            robot.localization.update();
            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), robot.localization.getPose());
            if (robot.opMode.gamepad1.triangle)
                Drawing.drawTargetPose(packet.fieldOverlay(), new Pose2d(robot.drive.targetX, robot.drive.targetY, new Rotation2d(robot.drive.targetH)));
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
        synchronizer.stop();
    }


    private void initSynchronizer() {


        // Drive to submersible to deposit preloaded specimen

        CRSplineTranslation spline1 = new CRSplineTranslation(0,
                new TranslationState(-24,63.5),
                new TranslationState(-12, 50),
                new TranslationState(-1, 37)
        );

        LinearRotation still = new LinearRotation(0,
                new RotationState(Math.toRadians(-90)),
                new RotationState(Math.toRadians(-90))
        );


        LinearLift liftPreload1 = new LinearLift(spline1.getStartTime(),
                new LiftState(0),
                new LiftState(1500)
        );

        LinearLift liftPreload2 = new LinearLift(liftPreload1.getEndTime(),
                new LiftState(1500),
                new LiftState(0)
        );

        // claw
        LinearClaw claw1 = new LinearClaw(liftPreload2.getStartTime()+.66,
                new ClawState(clawClosed),
                new ClawState(clawOpen)
        );

        //elbow
        LinearElbow elbowStill = new LinearElbow(claw1.getStartTime(), //goes down to sample
                new ElbowState(elbowUp),
                new ElbowState(elbowUp)
        );


        // Push sample into observation zone

        CRSplineTranslation splinePushSampleToObservation = new CRSplineTranslation(liftPreload2.getEndTime(),
                new TranslationState(-1, 37),
                new TranslationState(-33, 38),
                new TranslationState(-37, 16),
                new TranslationState(-46, 14),
                new TranslationState(-46, 58)
        );


        // Leave, wait, and re-enter observation zone

        LinearTranslation lineLeaveObservation = new LinearTranslation(splinePushSampleToObservation.getEndTime(),
                new TranslationState(-46, 58),
                new TranslationState(-48, 36)
        );

        LinearRotation rotateToObservation = new LinearRotation(lineLeaveObservation.getStartTime(),
                new RotationState(Math.toRadians(-90)),
                new RotationState(Math.toRadians(90))
        );


        // Pick up from observation zone

        LinearTranslation lineEnterObservation = new LinearTranslation(rotateToObservation.getEndTime()+1,
                new TranslationState(-48, 36),
                new TranslationState(-48, 48)
        );

        LinearElbow elbowDownObservation = new LinearElbow(lineEnterObservation.getEndTime(), //goes down to specimen
                new ElbowState(elbowUp),
                new ElbowState(elbowDown)
        );

        LinearClaw clawCloseObservation = new LinearClaw(elbowDownObservation.getEndTime()-0.1,
                new ClawState(clawOpen),
                new ClawState(clawClosed)
        );

        LinearElbow elbowUpObservation = new LinearElbow(clawCloseObservation.getEndTime()+0.2,
                new ElbowState(elbowDown),
                new ElbowState(elbowUp)
        );




        // Deposit specimen at submersible

        CRSplineTranslation splineObservationToSubmersible = new CRSplineTranslation(clawCloseObservation.getEndTime()+0.5,
                new TranslationState(-48, 48),
                new TranslationState(-12, 43.5),
                new TranslationState(-4, 37)
        );


        LinearRotation rotateToSubmersible = new LinearRotation(splineObservationToSubmersible.getStartTime()+0.1,
                new RotationState(Math.toRadians(90)),
                new RotationState(Math.toRadians(-90))
        );

        LinearLift liftCycleUp = new LinearLift(splineObservationToSubmersible.getStartTime(),
                new LiftState(0),
                new LiftState(1500)
        );

        LinearLift liftCycleDown = new LinearLift(splineObservationToSubmersible.getEndTime()-0.5,
                new LiftState(1500),
                new LiftState(0)
        );

        // claw
        LinearClaw clawCycleOpen = new LinearClaw(liftCycleDown.getStartTime()+.66,
                new ClawState(clawClosed),
                new ClawState(clawOpen)
        );






        // Pick up second specimen from observation zone

        CRSplineTranslation splineCycleToObservation = new CRSplineTranslation(liftCycleDown.getEndTime(),
                new TranslationState(-4, 37),
                new TranslationState(-24, 44),
                new TranslationState(-48, 48)
        );

        LinearRotation rotateCycleToObservation = new LinearRotation(splineCycleToObservation.getStartTime()+0.1,
                new RotationState(Math.toRadians(-90)),
                new RotationState(Math.toRadians(90))
        );

        LinearElbow elbowCycleDownObservation = new LinearElbow(splineCycleToObservation.getEndTime(), //goes down to specimen
                new ElbowState(elbowUp),
                new ElbowState(elbowDown)
        );

        LinearClaw clawCycleCloseObservation = new LinearClaw(elbowCycleDownObservation.getEndTime()-0.1,
                new ClawState(clawOpen),
                new ClawState(clawClosed)
        );

        LinearElbow elbowCycleUpObservation = new LinearElbow(clawCycleCloseObservation.getEndTime()+0.2,
                new ElbowState(elbowDown),
                new ElbowState(elbowUp)
        );



        // Deposit second specimen into submersible

        CRSplineTranslation splineCycleObservationToSubmersible = new CRSplineTranslation(clawCycleCloseObservation.getEndTime()+0.5,
                new TranslationState(-48, 48),
                new TranslationState(-12, 44),
                new TranslationState(-7, 37)
        );

        LinearRotation rotateCycleToSubmersible = new LinearRotation(splineCycleObservationToSubmersible.getStartTime()+0.1,
                new RotationState(Math.toRadians(90)),
                new RotationState(Math.toRadians(-90))
        );

        LinearLift liftCycle2Up = new LinearLift(splineCycleObservationToSubmersible.getStartTime(),
                new LiftState(0),
                new LiftState(1500)
        );

        LinearLift liftCycle2Down = new LinearLift(splineCycleObservationToSubmersible.getEndTime()-0.5,
                new LiftState(1500),
                new LiftState(0)
        );

        // claw
        LinearClaw clawCycle2Open = new LinearClaw(liftCycle2Down.getStartTime()+.66,
                new ClawState(clawClosed),
                new ClawState(clawOpen)
        );


        // Park in observation zone

        CRSplineTranslation splinePark = new CRSplineTranslation(liftCycle2Down.getEndTime(),
                new TranslationState(-7, 37),
                new TranslationState(-18, 48),
                new TranslationState(-48, 63.5)
        );




        // Create Plans

        TranslationPlan translationPlan = new TranslationPlan(robot,
                spline1,
                splinePushSampleToObservation,
                lineLeaveObservation,
                lineEnterObservation,
                splineObservationToSubmersible,
                splineCycleToObservation,
                splineCycleObservationToSubmersible,
                splinePark
        );

        RotationPlan rotationPlan = new RotationPlan(robot,
                still,
                rotateToObservation,
                rotateToSubmersible,
                rotateCycleToObservation,
                rotateCycleToSubmersible
        );

        LiftPlan liftPlan = new LiftPlan(robot,
                liftPreload1,
                liftPreload2,
                liftCycleUp,
                liftCycleDown,
                liftCycle2Up,
                liftCycle2Down
        );

        ClawPlan clawPlan = new ClawPlan(robot,
                claw1,
                clawCloseObservation,
                clawCycleOpen,
                clawCycleCloseObservation,
                clawCycle2Open
        );

        ElbowPlan elbowPlan = new ElbowPlan(robot,
                elbowStill,
                elbowDownObservation,
                elbowUpObservation,
                elbowCycleDownObservation,
                elbowCycleUpObservation
        );

        this.synchronizer = new Synchronizer(
                translationPlan,
                rotationPlan,
                liftPlan,
                elbowPlan,
                clawPlan
        );
    }

}
