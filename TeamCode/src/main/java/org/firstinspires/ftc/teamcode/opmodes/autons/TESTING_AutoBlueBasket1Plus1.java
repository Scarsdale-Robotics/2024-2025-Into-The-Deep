package org.firstinspires.ftc.teamcode.opmodes.autons;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
@Autonomous(name="[TESTING] Auto Blue Basket 1+1", group = "Autons")
public class TESTING_AutoBlueBasket1Plus1 extends LinearOpMode {

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
        this.robot = new RobotSystem(hardwareMap, new Pose2d(40, 63.5, new Rotation2d(Math.toRadians(-90))), false, this);
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

        // Place preloaded specimen

        CRSplineTranslation splinePreload = new CRSplineTranslation(0,
                new TranslationState(40,63.5),
                new TranslationState(14, 48),
                new TranslationState(10, 37)
        );

        LinearRotation still = new LinearRotation(new TimeSpan(0,1),
                new RotationState(Math.toRadians(-90)),
                new RotationState(Math.toRadians(-90))
        );

        LinearLift liftPreload1 = new LinearLift(new TimeSpan(splinePreload.getStartTime(), splinePreload.getEndTime()-0.5),
                new LiftState(0),
                new LiftState(1400)
        );

        LinearLift liftPreload2 = new LinearLift(splinePreload.getEndTime()-0.5,
                new LiftState(1400),
                new LiftState(-50)
        );

        LinearClaw claw1 = new LinearClaw(liftPreload2.getStartTime()+.575,
                new ClawState(clawClosed),
                new ClawState(clawOpen)
        );

        LinearElbow elbowStill = new LinearElbow(new TimeSpan(0,1),
                new ElbowState(elbowUp),
                new ElbowState(elbowUp)
        );


        // Pick up first tape mark sample

        CRSplineTranslation splineApproachSample1 = new CRSplineTranslation(claw1.getStartTime(),
                new TranslationState(10, 37),
                new TranslationState(40, 48),
                new TranslationState(48.5, 44)
        );

        LinearElbow elbowDownSample1 = new LinearElbow(splineApproachSample1.getEndTime()-1,
                new ElbowState(elbowUp),
                new ElbowState(elbowDown)
        );

        LinearClaw clawCloseSample1 = new LinearClaw(elbowDownSample1.getEndTime(),
                new ClawState(clawOpen),
                new ClawState(clawClosed)
        );



        // Drop first sample in basket

        LinearLift liftUpSample1 = new LinearLift(clawCloseSample1.getEndTime(),
                new LiftState(-50),
                new LiftState(4100)
        );

        LinearRotation rotateScoreSample1 = new LinearRotation(liftUpSample1.getStartTime(),
                new RotationState(Math.toRadians(-90)),
                new RotationState(Math.toRadians(45))
        );

        CRSplineTranslation splineScoreSample1 = new CRSplineTranslation(new TimeSpan(liftUpSample1.getStartTime(), liftUpSample1.getEndTime()+1),
                new TranslationState(48.5, 44),
                new TranslationState(50, 48),
                new TranslationState(54, 56)
        );

        LinearElbow elbowUpSample1 = new LinearElbow(liftUpSample1.getStartTime(),
                new ElbowState(elbowDown),
                new ElbowState(elbowUp)
        );

        LinearClaw clawOpenSample1 = new LinearClaw(splineScoreSample1.getEndTime(),
                new ClawState(clawClosed),
                new ClawState(clawOpen)
        );




        // Pick up second tape mark sample

        LinearLift liftDownSample2 = new LinearLift(clawOpenSample1.getEndTime(),
                new LiftState(4100),
                new LiftState(-50)
        );

        CRSplineTranslation splineApproachSample2 = new CRSplineTranslation(liftDownSample2.getTimeSpan(),
                new TranslationState(54, 56),
                new TranslationState(56, 48),
                new TranslationState(58.5, 44)
        );

        LinearRotation rotateApproachSample2 = new LinearRotation(new TimeSpan(liftDownSample2.getStartTime(), liftDownSample2.getEndTime()-0.5),
                new RotationState(Math.toRadians(45)),
                new RotationState(Math.toRadians(-90))
        );

        LinearElbow elbowDownSample2 = new LinearElbow(splineApproachSample2.getEndTime()-1,
                new ElbowState(elbowUp),
                new ElbowState(elbowDown)
        );

        LinearClaw clawCloseSample2 = new LinearClaw(elbowDownSample2.getEndTime(),
                new ClawState(clawOpen),
                new ClawState(clawClosed)
        );




        // Drop second sample in basket

        LinearLift liftUpSample2 = new LinearLift(clawCloseSample2.getEndTime(),
                new LiftState(-50),
                new LiftState(4100)
        );

        LinearRotation rotateScoreSample2 = new LinearRotation(liftUpSample2.getStartTime(),
                new RotationState(Math.toRadians(-90)),
                new RotationState(Math.toRadians(45))
        );

        CRSplineTranslation splineScoreSample2 = new CRSplineTranslation(new TimeSpan(liftUpSample2.getStartTime(), liftUpSample2.getEndTime()+1),
                new TranslationState(58.5, 44),
                new TranslationState(56, 48),
                new TranslationState(54, 56)
        );

        LinearElbow elbowUpSample2 = new LinearElbow(liftUpSample2.getStartTime(),
                new ElbowState(elbowDown),
                new ElbowState(elbowUp)
        );

        LinearClaw clawOpenSample2 = new LinearClaw(splineScoreSample2.getEndTime(),
                new ClawState(clawClosed),
                new ClawState(clawOpen)
        );




        // Pick up third tape mark sample

        LinearLift liftDownSample3 = new LinearLift(clawOpenSample2.getEndTime(),
                new LiftState(4100),
                new LiftState(-50)
        );

        CRSplineTranslation splineApproachSample3 = new CRSplineTranslation(liftDownSample3.getTimeSpan(),
                new TranslationState(54, 56),
                new TranslationState(56, 48),
                new TranslationState(57.5, 39.5)
        );

        LinearRotation rotateApproachSample3 = new LinearRotation(new TimeSpan(liftDownSample3.getStartTime(), liftDownSample3.getEndTime()-0.5),
                new RotationState(Math.toRadians(45)),
                new RotationState(Math.toRadians(-51))
        );

        LinearElbow elbowDownSample3 = new LinearElbow(splineApproachSample3.getEndTime()-1,
                new ElbowState(elbowUp),
                new ElbowState(elbowDown)
        );

        LinearClaw clawCloseSample3 = new LinearClaw(elbowDownSample3.getEndTime(),
                new ClawState(clawOpen),
                new ClawState(clawClosed)
        );




        // Drop third sample in basket

        LinearLift liftUpSample3 = new LinearLift(clawCloseSample3.getEndTime(),
                new LiftState(-50),
                new LiftState(4100)
        );

        LinearRotation rotateScoreSample3 = new LinearRotation(liftUpSample3.getStartTime(),
                new RotationState(Math.toRadians(-51)),
                new RotationState(Math.toRadians(45))
        );

        CRSplineTranslation splineScoreSample3 = new CRSplineTranslation(new TimeSpan(liftUpSample3.getStartTime(), liftUpSample3.getEndTime()+1),
                new TranslationState(57.5, 39.5),
                new TranslationState(56, 48),
                new TranslationState(54, 56)
        );

        LinearElbow elbowUpSample3 = new LinearElbow(liftUpSample3.getStartTime(),
                new ElbowState(elbowDown),
                new ElbowState(elbowUp)
        );

        LinearClaw clawOpenSample3 = new LinearClaw(splineScoreSample3.getEndTime(),
                new ClawState(clawClosed),
                new ClawState(clawOpen)
        );





        // Park and lower elbow/lift

        double splineParkStartTime = clawOpenSample3.getEndTime();
        CRSplineTranslation splinePark = new CRSplineTranslation(new TimeSpan(splineParkStartTime, splineParkStartTime+4),
                new TranslationState(54, 56),
                new TranslationState(42, 14),
                new TranslationState(24, 10)
        );

        double rotateParkStartTime = splinePark.getStartTime();
        LinearRotation rotatePark = new LinearRotation(new TimeSpan(rotateParkStartTime, rotateParkStartTime+2),
                new RotationState(Math.toRadians(45)),
                new RotationState(Math.toRadians(-90))
        );

        LinearLift liftPark = new LinearLift(splinePark.getStartTime(),
                new LiftState(4100),
                new LiftState(-50)
        );

        LinearElbow elbowPark = new LinearElbow(splinePark.getEndTime()-1.5, //goes down to sample
                new ElbowState(elbowUp),
                new ElbowState(elbowDown)
        );

        LinearClaw clawPark = new LinearClaw(elbowPark.getStartTime(),
                new ClawState(clawOpen),
                new ClawState(clawClosed)
        );





        // Create Plans

        translationPlan = new TranslationPlan(robot,
                splinePreload,
                splineApproachSample1,
                splineScoreSample1,
                splineApproachSample2,
                splineScoreSample2,
                splineApproachSample3,
                splineScoreSample3,
                splinePark
        );

        rotationPlan = new RotationPlan(robot,
                still,
                rotateScoreSample1,
                rotateApproachSample2,
                rotateScoreSample2,
                rotateApproachSample3,
                rotateScoreSample3,
                rotatePark
        );

        liftPlan = new LiftPlan(robot,
                liftPreload1,
                liftPreload2,
                liftUpSample1,
                liftDownSample2,
                liftUpSample2,
                liftDownSample3,
                liftUpSample3,
                liftPark
        );

        clawPlan = new ClawPlan(robot,
                claw1,
                clawCloseSample1,
                clawOpenSample1,
                clawCloseSample2,
                clawOpenSample2,
                clawCloseSample3,
                clawOpenSample3,
                clawPark
        );

        elbowPlan = new ElbowPlan(robot,
                elbowStill,
                elbowDownSample1,
                elbowUpSample1,
                elbowDownSample2,
                elbowUpSample2,
                elbowDownSample3,
                elbowUpSample3,
                elbowPark
        );


    }

}
