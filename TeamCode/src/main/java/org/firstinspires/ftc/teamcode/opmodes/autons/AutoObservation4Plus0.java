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
@Autonomous(name="Auto OBSERVATION 4+0", group = "Autons")
public class AutoObservation4Plus0 extends LinearOpMode {

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
    public static double elbowPartiallyUp = 0.64;
    public static double elbowDown = ElbowConstants.DOWN_POSITION;

    @Override
    public void runOpMode() throws InterruptedException {
        initSynchronizer();
        this.robot = new RobotSystem(hardwareMap, new Pose2d(-15.5, 63.5, new Rotation2d(Math.toRadians(-90))), false, this);
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

        // Set PIDF values
        TranslationPlan.kA = 0.18;
        TranslationPlan.kD = 1;
        TranslationPlan.kP = 8;
        TranslationPlan.kS = 0;
        TranslationPlan.kV = 1;

        RotationPlan.kA = 0.1;
        RotationPlan.kD = 0.5;
        RotationPlan.kP = 4;
        RotationPlan.kS = 0;
        RotationPlan.kV = 1;

        DriveConstants.MAX_ANGULAR_VELOCITY = 3.9;


        ///////////////////////////////////////
        // PLACE PRELOADED SPECIMEN  (STEP 1)//
        ///////////////////////////////////////

        TranslationConstants.MAX_VELOCITY = 40d;
        TranslationConstants.MAX_ACCELERATION = 54d;

        RotationConstants.MAX_ANGULAR_VELOCITY = 3.6;
        RotationConstants.MAX_ANGULAR_ACCELERATION = 7.2;

        LiftConstants.MAX_VELOCITY = 2200;
        LiftConstants.MAX_ACCELERATION = 8200;

        ClawConstants.MAX_VELOCITY = 15.111111111;
        ClawConstants.MAX_ACCELERATION = 30.22222;

        ElbowConstants.MAX_VELOCITY = 1.021739;
        ElbowConstants.MAX_ACCELERATION = 1.021739;

        CRSplineTranslation splinePreload = new CRSplineTranslation(0,
                new TranslationState(-15.5,63.5),
                new TranslationState(-10, 37)
        );

        LinearLift liftUpPreload = new LinearLift(new TimeSpan(splinePreload.getStartTime(), splinePreload.getEndTime()-0.3),
                new LiftState(0),
                new LiftState(1350)
        );

        LinearRotation rotationStillPreload = new LinearRotation(0,
                new RotationState(Math.toRadians(-90)),
                new RotationState(Math.toRadians(-90))
        );

        LinearElbow elbowStillPreload = new LinearElbow(0,
                new ElbowState(elbowUp),
                new ElbowState(elbowUp)
        );

        LinearLift liftDownPreload = new LinearLift(liftUpPreload.getEndTime(),
                new LiftState(1350),
                new LiftState(0)
        );

        LinearClaw clawOpenPreload = new LinearClaw(liftDownPreload.getStartTime()+.69,
                new ClawState(clawClosed),
                new ClawState(clawOpen)
        );


        /////////////////////////////////
        // GO TO BLUE SAMPLES (Step 2) //
        /////////////////////////////////

        TranslationConstants.MAX_VELOCITY = 0.5*40d;

        CRSplineTranslation splineApproachSamples = new CRSplineTranslation(clawOpenPreload.getEndTime(),
                new TranslationState(-10, 37),
                new TranslationState(-18, 39),
                new TranslationState(-38, 39.5)
        );

        LinearRotation rotationApproachSamples = new LinearRotation(splineApproachSamples.getStartTime(),
                new RotationState(Math.toRadians(-90)),
                new RotationState(Math.toRadians(-129))
        );

        LinearElbow elbowApproachSamples = new LinearElbow(splineApproachSamples.getStartTime(),
                new ElbowState(elbowUp),
                new ElbowState(elbowPartiallyUp)
        );

        /////////////////////////////////////////////////////
        // MOVE EACH SAMPLE INTO OBSERVATION ZONE (Step 3) //
        /////////////////////////////////////////////////////

        double splineMoveFirstSampleStartTime = splineApproachSamples.getEndTime()+0.4;
        CRSplineTranslation splineMoveFirstSample = new CRSplineTranslation(new TimeSpan(splineMoveFirstSampleStartTime, splineMoveFirstSampleStartTime+2.1),
                new TranslationState(-38, 39.5), // Pick up first
                new TranslationState(-42, 49), // Deposit first
                new TranslationState(-48, 39.5) // Pick up second
        );
        double[] splineMoveFirstSampleTimes = splineMoveFirstSample.getSegmentTimes();

        // Pick up first sample
        LinearElbow elbowDownFirstSample = new LinearElbow(splineMoveFirstSampleTimes[0]-0.4,
                new ElbowState(elbowPartiallyUp),
                new ElbowState(elbowDown)
        );

        LinearClaw clawCloseFirstSample = new LinearClaw(elbowDownFirstSample.getEndTime()-0.1,
                new ClawState(clawOpen),
                new ClawState(clawClosed)
        );

        // Drop first sample
        LinearRotation rotationMoveFirstSample = new LinearRotation(new TimeSpan(splineMoveFirstSampleTimes[0], splineMoveFirstSampleTimes[1]),
                new RotationState(Math.toRadians(-129)),
                new RotationState(Math.toRadians(-219))
        );

        LinearElbow elbowUpFirstSample = new LinearElbow(clawCloseFirstSample.getEndTime(),
                new ElbowState(elbowDown),
                new ElbowState(elbowPartiallyUp)
        );

        LinearClaw clawOpenFirstSample = new LinearClaw(rotationMoveFirstSample.getEndTime() - 0.3,
                new ClawState(clawClosed),
                new ClawState(clawOpen)
        );

        // Pick up second sample
        LinearRotation rotationApproachSecondSample = new LinearRotation(new TimeSpan(splineMoveFirstSampleTimes[1], splineMoveFirstSampleTimes[2]),
                new RotationState(Math.toRadians(-219)),
                new RotationState(Math.toRadians(-129))
        );

        LinearElbow elbowDownSecondSample = new LinearElbow(splineMoveFirstSampleTimes[2],
                new ElbowState(elbowPartiallyUp),
                new ElbowState(elbowDown)
        );

        LinearClaw clawCloseSecondSample = new LinearClaw(elbowDownSecondSample.getEndTime()-0.1,
                new ClawState(clawOpen),
                new ClawState(clawClosed)
        );


        double splineMoveSecondSampleStartTime = splineMoveFirstSample.getEndTime()+0.4;
        CRSplineTranslation splineMoveSecondSample = new CRSplineTranslation(new TimeSpan(splineMoveSecondSampleStartTime, splineMoveSecondSampleStartTime+2),
                new TranslationState(-48, 39.5), // Pick up second
                new TranslationState(-52, 49), // Deposit second
                new TranslationState( -48, 44) // Go to cycle starting position
        );
        double[] splineMoveSecondSampleTimes = splineMoveSecondSample.getSegmentTimes();

        // Drop second sample
        LinearRotation rotationMoveSecondSample = new LinearRotation(new TimeSpan(splineMoveSecondSampleTimes[0], splineMoveSecondSampleTimes[1]),
                new RotationState(Math.toRadians(-129)),
                new RotationState(Math.toRadians(-219))
        );

        LinearElbow elbowUpSecondSample = new LinearElbow(clawCloseSecondSample.getEndTime(),
                new ElbowState(elbowDown),
                new ElbowState(elbowPartiallyUp)
        );

        LinearClaw clawOpenSecondSample = new LinearClaw(rotationMoveSecondSample.getEndTime() - 0.3,
                new ClawState(clawClosed),
                new ClawState(clawOpen)
        );

        // Turn to cycle starting position
        LinearRotation rotationApproachCycleStart = new LinearRotation(new TimeSpan(splineMoveSecondSampleTimes[1], splineMoveSecondSampleTimes[2]),
                new RotationState(Math.toRadians(-219)),
                new RotationState(Math.toRadians(-270))
        );

        LinearElbow elbowUpWaitingForHP = new LinearElbow(clawOpenSecondSample.getEndTime(),
                new ElbowState(elbowPartiallyUp),
                new ElbowState(elbowUp)
        );


        /////////////////////////////////////
        // PICK UP FIRST SPECIMEN (Step 4) //
        /////////////////////////////////////

        LinearElbow elbowDownFirstSpecimen = new LinearElbow(elbowUpWaitingForHP.getEndTime()+0.5, // clip end of spline
                new ElbowState(elbowUp),
                new ElbowState(elbowDown)
        );

        LinearClaw clawCloseFirstSpecimen = new LinearClaw(elbowDownFirstSpecimen.getEndTime(), // clip end of elbow
                new ClawState(clawOpen),
                new ClawState(clawClosed)
        );

        ///////////////////////////////////
        // SCORE FIRST SPECIMEN (Step 5) //
        ///////////////////////////////////

        // Approach
        LinearTranslation lineScoreFirstSpecimen = new LinearTranslation(clawCloseFirstSpecimen.getEndTime(),
                new TranslationState(-48, 46),
                new TranslationState(-8, 37)
        );

        LinearRotation rotationScoreFirstSpecimen = new LinearRotation(new TimeSpan(lineScoreFirstSpecimen.getStartTime(), lineScoreFirstSpecimen.getEndTime()-0.2),
                new RotationState(Math.toRadians(90)),
                new RotationState(Math.toRadians(-90))
        );

        LinearLift liftUpFirstSpecimen = new LinearLift(lineScoreFirstSpecimen.getStartTime(),
                new LiftState(-50),
                new LiftState(1350)
        );

        LinearElbow elbowUpFirstSpecimen = new LinearElbow(lineScoreFirstSpecimen.getStartTime(),
                new ElbowState(elbowDown),
                new ElbowState(elbowUp)
        );

        // Score
        LinearLift liftDownFirstSpecimen = new LinearLift(lineScoreFirstSpecimen.getEndTime(),
                new LiftState(1350),
                new LiftState(-50)
        );

        LinearClaw clawOpenFirstSpecimen = new LinearClaw(liftDownFirstSpecimen.getStartTime()+.69,
                new ClawState(clawClosed),
                new ClawState(clawOpen)
        );

        //////////////////////////////////////
        // PICK UP SECOND SPECIMEN (Step 6) //
        //////////////////////////////////////

        LinearTranslation lineApproachSecondSpecimen = new LinearTranslation(clawOpenFirstSpecimen.getEndTime(),
                new TranslationState(-8, 37),
                new TranslationState(-48, 46)
        );

        LinearRotation rotationApproachSecondSpecimen = new LinearRotation(new TimeSpan(lineApproachSecondSpecimen.getStartTime(), lineApproachSecondSpecimen.getEndTime()-0.2),
                new RotationState(Math.toRadians(-90)),
                new RotationState(Math.toRadians(90))
        );

        LinearElbow elbowDownSecondSpecimen = new LinearElbow(lineApproachSecondSpecimen.getEndTime(), // clip end of rotation
                new ElbowState(elbowUp),
                new ElbowState(elbowDown)
        );

        LinearClaw clawCloseSecondSpecimen = new LinearClaw(elbowDownSecondSpecimen.getEndTime(), // clip end of elbow
                new ClawState(clawOpen),
                new ClawState(clawClosed)
        );

        ////////////////////////////////////
        // SCORE SECOND SPECIMEN (Step 7) //
        ////////////////////////////////////

        // Approach
        LinearTranslation lineScoreSecondSpecimen = new LinearTranslation(clawCloseSecondSpecimen.getEndTime(),
                new TranslationState(-48, 46),
                new TranslationState(-6, 37)
        );

        LinearRotation rotationScoreSecondSpecimen = new LinearRotation(new TimeSpan(lineScoreSecondSpecimen.getStartTime(), lineScoreSecondSpecimen.getEndTime()-0.2),
                new RotationState(Math.toRadians(90)),
                new RotationState(Math.toRadians(-90))
        );

        LinearLift liftUpSecondSpecimen = new LinearLift(lineScoreSecondSpecimen.getStartTime(),
                new LiftState(-50),
                new LiftState(1350)
        );

        LinearElbow elbowUpSecondSpecimen = new LinearElbow(lineScoreSecondSpecimen.getStartTime(),
                new ElbowState(elbowDown),
                new ElbowState(elbowUp)
        );

        // Score
        LinearLift liftDownSecondSpecimen = new LinearLift(lineScoreSecondSpecimen.getEndTime(),
                new LiftState(1350),
                new LiftState(-50)
        );

        LinearClaw clawOpenSecondSpecimen = new LinearClaw(liftDownSecondSpecimen.getStartTime()+.69,
                new ClawState(clawClosed),
                new ClawState(clawOpen)
        );

        /////////////////////////////////////
        // PICK UP THIRD SPECIMEN (Step 8) //
        /////////////////////////////////////

        LinearTranslation lineApproachThirdSpecimen = new LinearTranslation(clawOpenSecondSpecimen.getEndTime(),
                new TranslationState(-6, 37),
                new TranslationState(-48, 46)
        );

        LinearRotation rotationApproachThirdSpecimen = new LinearRotation(new TimeSpan(lineApproachThirdSpecimen.getStartTime(), lineApproachThirdSpecimen.getEndTime()-0.2),
                new RotationState(Math.toRadians(-90)),
                new RotationState(Math.toRadians(90))
        );

        LinearElbow elbowDownThirdSpecimen = new LinearElbow(lineApproachThirdSpecimen.getEndTime(), // clip end of rotation
                new ElbowState(elbowUp),
                new ElbowState(elbowDown)
        );

        LinearClaw clawCloseThirdSpecimen = new LinearClaw(elbowDownThirdSpecimen.getEndTime(), // clip end of elbow
                new ClawState(clawOpen),
                new ClawState(clawClosed)
        );

        ///////////////////////////////////
        // SCORE THIRD SPECIMEN (Step 9) //
        ///////////////////////////////////

        // Approach
        LinearTranslation lineScoreThirdSpecimen = new LinearTranslation(clawCloseThirdSpecimen.getEndTime(),
                new TranslationState(-48, 46),
                new TranslationState(-4, 37)
        );

        LinearRotation rotationScoreThirdSpecimen = new LinearRotation(new TimeSpan(lineScoreThirdSpecimen.getStartTime(), lineScoreThirdSpecimen.getEndTime()-0.2),
                new RotationState(Math.toRadians(90)),
                new RotationState(Math.toRadians(-90))
        );

        LinearLift liftUpThirdSpecimen = new LinearLift(lineScoreThirdSpecimen.getStartTime(),
                new LiftState(-50),
                new LiftState(1350)
        );

        LinearElbow elbowUpThirdSpecimen = new LinearElbow(lineScoreThirdSpecimen.getStartTime(),
                new ElbowState(elbowDown),
                new ElbowState(elbowUp)
        );

        // Score
        LinearLift liftDownThirdSpecimen = new LinearLift(lineScoreThirdSpecimen.getEndTime(),
                new LiftState(1350),
                new LiftState(-50)
        );

        LinearClaw clawOpenThirdSpecimen = new LinearClaw(liftDownThirdSpecimen.getStartTime()+.69,
                new ClawState(clawClosed),
                new ClawState(clawOpen)
        );

        ////////////////////////////////////////
        // PARK IN OBSERVATION ZONE (Step 10) //
        ////////////////////////////////////////

        TranslationConstants.MAX_VELOCITY = 40d;

        LinearTranslation linePark = new LinearTranslation(clawOpenThirdSpecimen.getEndTime(),
                new TranslationState(-4, 37),
                new TranslationState(-42, 63.5)
        );

        LinearClaw clawPark = new LinearClaw(linePark.getStartTime(),
                new ClawState(clawOpen),
                new ClawState(clawClosed)
        );

        LinearElbow elbowPark = new LinearElbow(linePark.getStartTime(),
                new ElbowState(elbowUp),
                new ElbowState(elbowUp-0.2)
        );




        //////////////////
        // CREATE PLANS //
        //////////////////

        translationPlan = new TranslationPlan(robot,
                // PRELOAD
                splinePreload,

                // MOVING SAMPLES
                splineApproachSamples,
                splineMoveFirstSample,
                splineMoveSecondSample,

                // SCORING SPECIMENS
                lineScoreFirstSpecimen,

                lineApproachSecondSpecimen,
                lineScoreSecondSpecimen,

                lineApproachThirdSpecimen,
                lineScoreThirdSpecimen,

                linePark
        );

        rotationPlan = new RotationPlan(robot,
                // PRELOAD
                rotationStillPreload,

                // MOVING SAMPLES
                rotationApproachSamples,
                rotationMoveFirstSample,
                rotationApproachSecondSample,
                rotationMoveSecondSample,
                rotationApproachCycleStart,

                // SCORING SPECIMENS
                rotationScoreFirstSpecimen,

                rotationApproachSecondSpecimen,
                rotationScoreSecondSpecimen,

                rotationApproachThirdSpecimen,
                rotationScoreThirdSpecimen
        );

        liftPlan = new LiftPlan(robot,
                // PRELOAD
                liftUpPreload,
                liftDownPreload,

                // SCORING SPECIMENS
                liftUpFirstSpecimen,
                liftDownFirstSpecimen,

                liftUpSecondSpecimen,
                liftDownSecondSpecimen,

                liftUpThirdSpecimen,
                liftDownThirdSpecimen
        );


        clawPlan = new ClawPlan(robot,
                // PRELOAD
                clawOpenPreload,

                // MOVING SAMPLES
                clawCloseFirstSample,
                clawOpenFirstSample,
                clawCloseSecondSample,
                clawOpenSecondSample,

                // SCORING SPECIMENS
                clawCloseFirstSpecimen,
                clawOpenFirstSpecimen,

                clawCloseSecondSpecimen,
                clawOpenSecondSpecimen,

                clawCloseThirdSpecimen,
                clawOpenThirdSpecimen,

                clawPark
        );


        elbowPlan = new ElbowPlan(robot,
                // PRELOAD
                elbowStillPreload,

                // MOVING SAMPLES
                elbowApproachSamples,

                elbowDownFirstSample,
                elbowUpFirstSample,
                elbowDownSecondSample,
                elbowUpSecondSample,
                elbowUpWaitingForHP,

                // SCORING SPECIMENS
                elbowDownFirstSpecimen,
                elbowUpFirstSpecimen,

                elbowDownSecondSpecimen,
                elbowUpSecondSpecimen,

                elbowDownThirdSpecimen,
                elbowUpThirdSpecimen,

                elbowPark
        );

    }

}
