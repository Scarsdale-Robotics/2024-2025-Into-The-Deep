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
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.TimeSpan;
import org.firstinspires.ftc.teamcode.synchropather.systems.claw.ClawConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.claw.ClawPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.claw.ClawState;
import org.firstinspires.ftc.teamcode.synchropather.systems.claw.movements.LinearClaw;
import org.firstinspires.ftc.teamcode.synchropather.systems.elbow.ElbowConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.elbow.ElbowPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.elbow.ElbowState;
import org.firstinspires.ftc.teamcode.synchropather.systems.elbow.movements.LinearElbow;
import org.firstinspires.ftc.teamcode.synchropather.systems.lift.LiftConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.lift.LiftPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.lift.LiftState;
import org.firstinspires.ftc.teamcode.synchropather.systems.lift.movements.LinearLift;
import org.firstinspires.ftc.teamcode.synchropather.systems.rotation.RotationConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.rotation.RotationPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.rotation.RotationState;
import org.firstinspires.ftc.teamcode.synchropather.systems.rotation.movements.LinearRotation;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.TranslationConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.TranslationPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.TranslationState;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.movements.CRSplineTranslation;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.movements.LinearTranslation;

//@Disabled
@Autonomous(name="Auto Blue Observation 1+1", group = "Autons")
public class AutoBlueObservation1Plus1 extends LinearOpMode {

    RobotSystem robot;
    Synchronizer synchronizer;
    private TranslationPlan translationPlan;
    private RotationPlan rotationPlan;
    private LiftPlan liftPlan;
    private ClawPlan clawPlan;
    private ElbowPlan elbowPlan;

    public static double clawOpen = ClawConstants.OPEN_POSITION;
    public static double clawClosed = ClawConstants.CLOSED_POSITION;

    public static double elbowUp = ElbowConstants.UP_POSITION;
    public static double elbowDown = ElbowConstants.DOWN_POSITION;


    @Override
    public void runOpMode() throws InterruptedException {
        initSynchronizer();
        this.robot = new RobotSystem(hardwareMap, new Pose2d(-24, 63.5, new Rotation2d(Math.toRadians(-90))), false, this);
        this.translationPlan.setRobot(robot);
        this.rotationPlan.setRobot(robot);
        this.liftPlan.setRobot(robot);
        this.clawPlan.setRobot(robot);
        this.elbowPlan.setRobot(robot);
        this.synchronizer = new Synchronizer(
                translationPlan,
                rotationPlan,
                liftPlan,
                elbowPlan,
                clawPlan
        );

        synchronizer.start();
        while (opModeIsActive() && synchronizer.update()) {
            robot.localization.update();
            robot.logOdometry();
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

        DriveConstants.MAX_ANGULAR_VELOCITY = 3.6;
        DriveConstants.MAX_ANGULAR_ACCELERATION = 4;

        TranslationConstants.MAX_VELOCITY = 0.5*40d;
        TranslationConstants.MAX_ACCELERATION = 0.5*54d;

        RotationConstants.MAX_ANGULAR_VELOCITY = 0.65*3.6;
        RotationConstants.MAX_ANGULAR_ACCELERATION = 0.65*4;

        LiftConstants.MAX_VELOCITY = 2200;
        LiftConstants.MAX_ACCELERATION = 2200;

        ClawConstants.MAX_VELOCITY = 15.111111111;
        ClawConstants.MAX_ACCELERATION = 30.22222;

        ElbowConstants.MAX_VELOCITY = 1.021739;
        ElbowConstants.MAX_ACCELERATION = 1.021739;

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
                new LiftState(1350)
        );

        LinearLift liftPreload2 = new LinearLift(spline1.getEndTime()-0.5,
                new LiftState(1350),
                new LiftState(-50)
        );

        LinearClaw claw1 = new LinearClaw(liftPreload2.getStartTime()+.66,
                new ClawState(clawClosed),
                new ClawState(clawOpen)
        );

        LinearElbow elbowStill = new LinearElbow(claw1.getStartTime(),
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
                new LiftState(-50),
                new LiftState(1350)
        );

        LinearLift liftCycleDown = new LinearLift(splineObservationToSubmersible.getEndTime()-0.5,
                new LiftState(1350),
                new LiftState(-50)
        );

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

        LinearElbow elbowPark = new LinearElbow(splinePark.getEndTime()-1.5, //goes down to sample
                new ElbowState(elbowUp),
                new ElbowState(elbowUp-0.2)
        );

        LinearClaw clawPark = new LinearClaw(elbowPark.getStartTime(),
                new ClawState(clawOpen),
                new ClawState(clawClosed)
        );




        // Create Plans

        translationPlan = new TranslationPlan(robot,
                spline1,
                lineToObservation,
                splineObservationToSubmersible,
                splinePark
        );

        rotationPlan = new RotationPlan(robot,
                still,
                rotateToObservation,
                rotateToSubmersible
        );

        liftPlan = new LiftPlan(robot,
                liftPreload1,
                liftPreload2,
                liftCycleUp,
                liftCycleDown
        );

        clawPlan = new ClawPlan(robot,
                claw1,
                clawCloseObservation,
                clawCycleOpen,
                clawPark
        );

        elbowPlan = new ElbowPlan(robot,
                elbowStill,
                elbowDownObservation,
                elbowUpObservation,
                elbowPark
        );

    }

}