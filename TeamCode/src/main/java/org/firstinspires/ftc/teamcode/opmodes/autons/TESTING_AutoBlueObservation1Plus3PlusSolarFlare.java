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
@Autonomous(name="[SF COLLAB.] Auto Blue Observation 1+3+sample for Solar Flare", group = "Autons")
public class TESTING_AutoBlueObservation1Plus3PlusSolarFlare extends LinearOpMode {

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
                new TranslationState(-4, 37)
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
                new LiftState(-50)
        );

        LinearClaw clawOpenPreload = new LinearClaw(liftDownPreload.getStartTime()+.69,
                new ClawState(clawClosed),
                new ClawState(clawOpen)
        );



        ///////////////////////////////////////////////
        // TRANSFER SAMPLE TO SOLAR FLARE (Step 1.5) //
        ///////////////////////////////////////////////

        TranslationConstants.MAX_VELOCITY = 0.8*40d;

        // Approach sample
        LinearTranslation lineApproachTransferSample = new LinearTranslation(clawOpenPreload.getEndTime(),
                new TranslationState(-4, 37),
                new TranslationState(-31, 58)
        );

        LinearRotation rotationApproachTransferSample = new LinearRotation(lineApproachTransferSample.getTrimmedTimeSpan(0, 0.25),
                new RotationState(Math.toRadians(-90)),
                new RotationState(Math.toRadians(-180))
        );

        LinearElbow elbowApproachTransferSample = new LinearElbow(lineApproachTransferSample.getStartTime(),
                new ElbowState(elbowUp),
                new ElbowState(elbowPartiallyUp)
        );

        // Grab sample
        LinearElbow elbowDownGrabTransferSample = new LinearElbow(lineApproachTransferSample.getEndTime()-0.2,
                new ElbowState(elbowPartiallyUp),
                new ElbowState(elbowDown)
        );

        LinearClaw clawCloseTransferSample = new LinearClaw(elbowDownGrabTransferSample.getEndTime()-0.1,
                new ClawState(clawOpen),
                new ClawState(clawClosed)
        );

        LinearElbow elbowUpGrabTransferSample = new LinearElbow(clawCloseTransferSample.getEndTime(),
                new ElbowState(elbowDown),
                new ElbowState(elbowUp)
        );

        // Deposit sample
        TranslationConstants.MAX_VELOCITY = 0.5*40d;

        LinearTranslation lineDepositTransferSample = new LinearTranslation(elbowUpGrabTransferSample.getStartTime(),
                new TranslationState(-31, 58),
                new TranslationState(-5, 48)
        );

        LinearRotation rotationDepositTransferSample = new LinearRotation(lineDepositTransferSample.getTrimmedTimeSpan(0, 0.25),
                new RotationState(Math.toRadians(-180)),
                new RotationState(Math.toRadians(0))
        );

        LinearElbow elbowDownDepositTransferSample = new LinearElbow(lineDepositTransferSample.getEndTime()-0.4,
                new ElbowState(elbowUp),
                new ElbowState(elbowDown)
        );

        LinearClaw clawOpenTransferSample = new LinearClaw(elbowDownDepositTransferSample.getEndTime()-0.1,
                new ClawState(clawClosed),
                new ClawState(clawOpen)
        );

        System.out.println(clawOpenTransferSample.getEndTime());



        /////////////////////////////////
        // GO TO BLUE SAMPLES (Step 2) //
        /////////////////////////////////

        TranslationConstants.MAX_VELOCITY = 0.6*40d;

        LinearTranslation lineApproachSamples = new LinearTranslation(clawOpenTransferSample.getEndTime()+0.1,
                new TranslationState(-5, 48),
                new TranslationState(-49, 44)
        );

        LinearRotation rotationApproachSamples = new LinearRotation(lineApproachSamples.getTrimmedTimeSpan(0, 0.5),
                new RotationState(Math.toRadians(0)),
                new RotationState(Math.toRadians(-90))
        );

        LinearElbow elbowApproachSamples = new LinearElbow(lineApproachSamples.getStartTime()-0.1,
                new ElbowState(elbowDown),
                new ElbowState(elbowPartiallyUp)
        );


        /////////////////////////////////////////////////////
        // MOVE EACH SAMPLE INTO OBSERVATION ZONE (Step 3) //
        /////////////////////////////////////////////////////

        TranslationConstants.MAX_VELOCITY = 0.5*40d;

        // Pick up first sample
        LinearElbow elbowDownFirstSample = new LinearElbow(lineApproachSamples.getEndTime()-0.3,
                new ElbowState(elbowPartiallyUp),
                new ElbowState(elbowDown)
        );

        LinearClaw clawCloseFirstSample = new LinearClaw(elbowDownFirstSample.getEndTime()-0.1,
                new ClawState(clawOpen),
                new ClawState(clawClosed)
        );

        // Drop first sample
        double splineMoveFirstSampleStartTime = lineApproachSamples.getEndTime();
        CRSplineTranslation splineMoveFirstSample = new CRSplineTranslation(new TimeSpan(splineMoveFirstSampleStartTime, splineMoveFirstSampleStartTime+2.25),
                new TranslationState(-49, 44), // Pick up first
                new TranslationState(-53.75, 44), // Deposit first
                new TranslationState(-58.5, 44) // Approach second
        );
        double[] splineMoveFirstSampleTimes = splineMoveFirstSample.getSegmentTimes();

        LinearRotation rotationMoveFirstSample = new LinearRotation(splineMoveFirstSample.getTimeSpan(),
                new RotationState(Math.toRadians(-90)),
                new RotationState(Math.toRadians(-450))
        );

        LinearElbow elbowUpFirstSample = new LinearElbow(clawCloseFirstSample.getEndTime(),
                new ElbowState(elbowDown),
                new ElbowState(elbowPartiallyUp)
        );

        LinearClaw clawOpenFirstSample = new LinearClaw(splineMoveFirstSampleTimes[1] - 0.3,
                new ClawState(clawClosed),
                new ClawState(clawOpen)
        );

        // Pick up second sample

        LinearElbow elbowDownSecondSample = new LinearElbow(splineMoveFirstSampleTimes[2] - 0.4,
                new ElbowState(elbowPartiallyUp),
                new ElbowState(elbowDown)
        );

        LinearClaw clawCloseSecondSample = new LinearClaw(elbowDownSecondSample.getEndTime() - 0.1,
                new ClawState(clawOpen),
                new ClawState(clawClosed)
        );


        double splineMoveSecondSampleStartTime = splineMoveFirstSample.getEndTime();
        CRSplineTranslation splineMoveSecondSample = new CRSplineTranslation(new TimeSpan(splineMoveSecondSampleStartTime, splineMoveSecondSampleStartTime+2.75),
                new TranslationState(-58.5, 44), // Pick up second
                new TranslationState(-52, 44), // Deposit second
                new TranslationState( -48, 41) // Go to cycle starting position
        );
        double[] splineMoveSecondSampleTimes = splineMoveSecondSample.getSegmentTimes();

        // Drop second sample
        LinearRotation rotationMoveSecondSample = new LinearRotation(new TimeSpan(splineMoveSecondSampleTimes[0], splineMoveSecondSampleTimes[1]),
                new RotationState(Math.toRadians(-90)),
                new RotationState(Math.toRadians(90))
        );

        LinearElbow elbowUpSecondSample = new LinearElbow(clawCloseSecondSample.getEndTime(),
                new ElbowState(elbowDown),
                new ElbowState(elbowPartiallyUp)
        );

        LinearClaw clawOpenSecondSample = new LinearClaw(rotationMoveSecondSample.getEndTime() - 0.3,
                new ClawState(clawClosed),
                new ClawState(clawOpen)
        );

        LinearElbow elbowUpWaitingForHP = new LinearElbow(clawOpenSecondSample.getEndTime(),
                new ElbowState(elbowPartiallyUp),
                new ElbowState(elbowUp)
        );


        /////////////////////////////////////
        // PICK UP FIRST SPECIMEN (Step 4) //
        /////////////////////////////////////

        LinearElbow elbowDownFirstSpecimen = new LinearElbow(splineMoveSecondSample.getEndTime()-0.1, // clip end of spline
                new ElbowState(elbowUp),
                new ElbowState(elbowDown)
        );

        LinearClaw clawCloseFirstSpecimen = new LinearClaw(elbowDownFirstSpecimen.getEndTime()-0.1, // clip end of elbow
                new ClawState(clawOpen),
                new ClawState(clawClosed)
        );

        ///////////////////////////////////
        // SCORE FIRST SPECIMEN (Step 5) //
        ///////////////////////////////////

        // Approach
        LinearTranslation lineScoreFirstSpecimen = new LinearTranslation(clawCloseFirstSpecimen.getEndTime(),
                new TranslationState(-48, 41),
                new TranslationState(-6, 37)
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
        LinearLift liftDownFirstSpecimen = new LinearLift(lineScoreFirstSpecimen.getEndTime()-0.2,
                new LiftState(1350),
                new LiftState(-50)
        );

        LinearClaw clawOpenFirstSpecimen = new LinearClaw(liftDownFirstSpecimen.getStartTime()+.35,
                new ClawState(clawClosed),
                new ClawState(clawOpen)
        );

        //////////////////////////////////////
        // PICK UP SECOND SPECIMEN (Step 6) //
        //////////////////////////////////////

        LinearTranslation lineApproachSecondSpecimen = new LinearTranslation(clawOpenFirstSpecimen.getEndTime(),
                new TranslationState(-6, 37),
                new TranslationState(-48, 41)
        );

        LinearRotation rotationApproachSecondSpecimen = new LinearRotation(new TimeSpan(lineApproachSecondSpecimen.getStartTime(), lineApproachSecondSpecimen.getEndTime()-0.2),
                new RotationState(Math.toRadians(-90)),
                new RotationState(Math.toRadians(90))
        );

        LinearElbow elbowDownSecondSpecimen = new LinearElbow(lineApproachSecondSpecimen.getEndTime()-0.8, // clip end of rotation
                new ElbowState(elbowUp),
                new ElbowState(elbowDown)
        );

        LinearClaw clawCloseSecondSpecimen = new LinearClaw(elbowDownSecondSpecimen.getEndTime()-0.1, // clip end of elbow
                new ClawState(clawOpen),
                new ClawState(clawClosed)
        );

        ////////////////////////////////////
        // SCORE SECOND SPECIMEN (Step 7) //
        ////////////////////////////////////

        // Approach
        LinearTranslation lineScoreSecondSpecimen = new LinearTranslation(clawCloseSecondSpecimen.getEndTime(),
                new TranslationState(-48, 41),
                new TranslationState(-8, 37)
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
        LinearLift liftDownSecondSpecimen = new LinearLift(lineScoreSecondSpecimen.getEndTime()-0.2,
                new LiftState(1350),
                new LiftState(-50)
        );

        LinearClaw clawOpenSecondSpecimen = new LinearClaw(liftDownSecondSpecimen.getStartTime()+.35,
                new ClawState(clawClosed),
                new ClawState(clawOpen)
        );

        /////////////////////////////////////
        // PICK UP THIRD SPECIMEN (Step 8) //
        /////////////////////////////////////

        LinearTranslation lineApproachThirdSpecimen = new LinearTranslation(clawOpenSecondSpecimen.getEndTime(),
                new TranslationState(-8, 37),
                new TranslationState(-48, 41)
        );

        LinearRotation rotationApproachThirdSpecimen = new LinearRotation(new TimeSpan(lineApproachThirdSpecimen.getStartTime(), lineApproachThirdSpecimen.getEndTime()-0.2),
                new RotationState(Math.toRadians(-90)),
                new RotationState(Math.toRadians(90))
        );

        LinearElbow elbowDownThirdSpecimen = new LinearElbow(lineApproachThirdSpecimen.getEndTime()-0.8, // clip end of rotation
                new ElbowState(elbowUp),
                new ElbowState(elbowDown)
        );

        LinearClaw clawCloseThirdSpecimen = new LinearClaw(elbowDownThirdSpecimen.getEndTime()-0.1, // clip end of elbow
                new ClawState(clawOpen),
                new ClawState(clawClosed)
        );

        ///////////////////////////////////
        // SCORE THIRD SPECIMEN (Step 9) //
        ///////////////////////////////////

        // Approach
        LinearTranslation lineScoreThirdSpecimen = new LinearTranslation(clawCloseThirdSpecimen.getEndTime(),
                new TranslationState(-48, 41),
                new TranslationState(-10, 37)
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
        LinearLift liftDownThirdSpecimen = new LinearLift(lineScoreThirdSpecimen.getEndTime()-0.2,
                new LiftState(1350),
                new LiftState(-50)
        );

        LinearClaw clawOpenThirdSpecimen = new LinearClaw(liftDownThirdSpecimen.getStartTime()+.35,
                new ClawState(clawClosed),
                new ClawState(clawOpen)
        );

        ////////////////////////////////////////
        // PARK IN OBSERVATION ZONE (Step 10) //
        ////////////////////////////////////////

        TranslationConstants.MAX_VELOCITY = 40d;

        LinearTranslation linePark = new LinearTranslation(clawOpenThirdSpecimen.getEndTime(),
                new TranslationState(-10, 37),
                new TranslationState(-48, 63.5)
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

                // TRANSFER SAMPLE TO SOLAR FLARE
                lineApproachTransferSample,
                lineDepositTransferSample,

                // MOVING SAMPLES
                lineApproachSamples,
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

                // TRANSFER SAMPLE TO SOLAR FLARE
                rotationApproachTransferSample,
                rotationDepositTransferSample,

                // MOVING SAMPLES
                rotationApproachSamples,
                rotationMoveFirstSample,
                rotationMoveSecondSample,

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

                // TRANSFER SAMPLE TO SOLAR FLARE
                clawCloseTransferSample,
                clawOpenTransferSample,

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

                // TRANSFER SAMPLE TO SOLAR FLARE
                elbowApproachTransferSample,
                elbowDownGrabTransferSample,
                elbowUpGrabTransferSample,
                elbowDownDepositTransferSample,

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
