package org.firstinspires.ftc.teamcode.opmodes.autons;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotSystem;
import org.firstinspires.ftc.teamcode.opmodes.calibration.Drawing;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.Synchronizer;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.TimeSpan;
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
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.TranslationPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.TranslationState;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.movements.CRSplineTranslation;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.movements.LinearTranslation;

//@Disabled
@Autonomous(name="[DISABLED] Auto Blue Observation 1+3", group = "Autons")
public class DISABLED_AutoBlueObservation1Plus3 extends LinearOpMode {

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
                new TranslationState(-5, 37)
        );

        LinearRotation still = new LinearRotation(0,
                new RotationState(Math.toRadians(-90)),
                new RotationState(Math.toRadians(-90))
        );


        LinearLift liftPreload1 = new LinearLift(new TimeSpan(spline1.getStartTime(), spline1.getEndTime()-0.5),
                new LiftState(0),
                new LiftState(1500)
        );

        LinearLift liftPreload2 = new LinearLift(spline1.getEndTime()-0.5,
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


        // Pick up from observation zone

        LinearTranslation lineToObservation = new LinearTranslation(liftPreload2.getEndTime(),
                new TranslationState(-5, 37),
                new TranslationState(-48, 48)
        );

        LinearRotation rotateToObservation = new LinearRotation(lineToObservation.getStartTime()+0.5,
                new RotationState(Math.toRadians(-90)),
                new RotationState(Math.toRadians(90))
        );

        LinearElbow elbowDownObservation = new LinearElbow(lineToObservation.getEndTime(), //goes down to specimen
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
                new TranslationState(-12, 48),
                new TranslationState(-10, 37)
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


        // Park in observation zone

        CRSplineTranslation splinePark = new CRSplineTranslation(liftCycleDown.getEndTime(),
                new TranslationState(-10, 37),
                new TranslationState(-18, 48),
                new TranslationState(-48, 63.5)
        );




        // Create Plans

        TranslationPlan translationPlan = new TranslationPlan(robot,
                spline1,
                lineToObservation,
                splineObservationToSubmersible,
                splinePark
        );

        RotationPlan rotationPlan = new RotationPlan(robot,
                still,
                rotateToObservation,
                rotateToSubmersible
        );

        LiftPlan liftPlan = new LiftPlan(robot,
                liftPreload1,
                liftPreload2,
                liftCycleUp,
                liftCycleDown
        );

        ClawPlan clawPlan = new ClawPlan(robot,
                claw1,
                clawCloseObservation,
                clawCycleOpen
        );

        ElbowPlan elbowPlan = new ElbowPlan(robot,
                elbowStill,
                elbowDownObservation,
                elbowUpObservation
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
