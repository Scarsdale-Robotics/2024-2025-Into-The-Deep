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
@Autonomous(name="[TESTING] Auto Blue Observation 1+4", group = "Autons")
public class TESTING_AutoBlueObservation1Plus4 extends LinearOpMode {

    RobotSystem robot;
    Synchronizer synchronizer;

    public static double clawOpen = ClawConstants.OPEN_POSITION;
    public static double clawClosed = ClawConstants.CLOSED_POSITION;

    public static double elbowUp = ElbowConstants.UP_POSITION;
    public static double elbowPartiallyUp = 0.34;
    public static double elbowDown = ElbowConstants.DOWN_POSITION;

    @Override
    public void runOpMode() throws InterruptedException {
        this.robot = new RobotSystem(hardwareMap, new Pose2d(-24, 63.5, new Rotation2d(Math.toRadians(-90))), false, this);
        robot.inDep.setClawPosition(clawClosed);
        robot.inDep.setElbowPosition(elbowUp-0.04);
        initSynchronizer();

        waitForStart();

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


        // Set kinematic constraints

        TranslationConstants.MAX_VELOCITY = 0.5*40d;
        TranslationConstants.MAX_ACCELERATION = 54d;

        RotationConstants.MAX_ANGULAR_VELOCITY = 3.6;
        RotationConstants.MAX_ANGULAR_ACCELERATION = 7.2;

        LiftConstants.MAX_VELOCITY = 2200;
        LiftConstants.MAX_ACCELERATION = 2200;

        ClawConstants.MAX_VELOCITY = 8;
        ClawConstants.MAX_ACCELERATION = 16;

        ElbowConstants.MAX_VELOCITY = 1;
        ElbowConstants.MAX_ACCELERATION = 1;


        // Set tuning variables
        double elbowClipDrive = 0.1;
        double clawClipElbow = 0.1;



        //////////////////////////////
        // PLACE PRELOADED SPECIMEN //
        //////////////////////////////

        LinearLift liftUpPreload = new LinearLift(0,
                new LiftState(0),
                new LiftState(1350)
        );

        CRSplineTranslation splinePreload = new CRSplineTranslation(new TimeSpan(liftUpPreload.getStartTime(), liftUpPreload.getEndTime()+0.3),
                new TranslationState(-15.5,63.5),
                new TranslationState(-10, 37)
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

        LinearClaw clawOpenPreload = new LinearClaw(liftDownPreload.getStartTime()+.5,
                new ClawState(clawClosed),
                new ClawState(clawOpen)
        );



        ////////////////////////
        // GO TO BLUE SAMPLES //
        ////////////////////////

        CRSplineTranslation splineApproachSamples = new CRSplineTranslation(clawOpenPreload.getEndTime(),
                new TranslationState(-10, 37),
                new TranslationState(-18, 39),
                new TranslationState(-37.5, 38.5)
        );

        LinearRotation rotationApproachSamples = new LinearRotation(splineApproachSamples.getStartTime(),
                new RotationState(Math.toRadians(-90)),
                new RotationState(Math.toRadians(-129))
        );

        LinearElbow elbowApproachSamples = new LinearElbow(splineApproachSamples.getStartTime(),
                new ElbowState(elbowUp),
                new ElbowState(elbowPartiallyUp)
        );
//        System.out.println("elbowApproachSamples.getDuration(): " + elbowApproachSamples.getDuration());




        ////////////////////////////////////////////
        // MOVE EACH SAMPLE INTO OBSERVATION ZONE //
        ////////////////////////////////////////////

        double splineMoveSamplesStartTime = splineApproachSamples.getEndTime();
        CRSplineTranslation splineMoveSamples = new CRSplineTranslation(new TimeSpan(splineMoveSamplesStartTime, splineMoveSamplesStartTime+5.4),
                new TranslationState(-37.5, 38.5), // Pick up first
                new TranslationState(-40, 49), // Deposit first
                new TranslationState(-47.5, 38.5), // Pick up second
                new TranslationState(-48, 44),
                new TranslationState(-52.25, 49), // Deposit second
                new TranslationState(-52, 44),
                new TranslationState(-57.5, 38.5), // Pick up third
                new TranslationState(-53, 48),
                new TranslationState( -48, 41) // Deposit third

        );
        double[] splineMoveSamplesTimes = splineMoveSamples.getSegmentTimes();
//		for (int i = 1; i < splineMoveSamplesTimes.length; i++) {
//			System.out.println("Segment " + i + ": " + (splineMoveSamplesTimes[i] - splineMoveSamplesTimes[i-1]) + "s");
//		}

        // Pick up first sample
        LinearElbow elbowDownFirstSample = new LinearElbow(splineMoveSamplesTimes[0]-elbowClipDrive,
                new ElbowState(elbowPartiallyUp),
                new ElbowState(elbowDown)
        );

        LinearClaw clawCloseFirstSample = new LinearClaw(elbowDownFirstSample.getEndTime()-clawClipElbow,
                new ClawState(clawOpen),
                new ClawState(clawClosed)
        );

        // Drop first sample
        LinearRotation rotationMoveFirstSample = new LinearRotation(new TimeSpan(splineMoveSamplesTimes[0], splineMoveSamplesTimes[1]),
                new RotationState(Math.toRadians(-129)),
                new RotationState(Math.toRadians(-219))
        );

        LinearElbow elbowUpFirstSample = new LinearElbow(clawCloseFirstSample.getEndTime(),
                new ElbowState(elbowDown),
                new ElbowState(elbowPartiallyUp)
        );

        LinearClaw clawOpenFirstSample = new LinearClaw(rotationMoveFirstSample.getEndTime(),
                new ClawState(clawClosed),
                new ClawState(clawOpen)
        );

        // Pick up second sample
        LinearRotation rotationApproachSecondSample = new LinearRotation(new TimeSpan(splineMoveSamplesTimes[1], splineMoveSamplesTimes[2]),
                new RotationState(Math.toRadians(-219)),
                new RotationState(Math.toRadians(-129))
        );

        LinearElbow elbowDownSecondSample = new LinearElbow(splineMoveSamplesTimes[2]-elbowClipDrive,
                new ElbowState(elbowPartiallyUp),
                new ElbowState(elbowDown)
        );

        LinearClaw clawCloseSecondSample = new LinearClaw(elbowDownSecondSample.getEndTime()-clawClipElbow,
                new ClawState(clawOpen),
                new ClawState(clawClosed)
        );

        // Drop second sample
        LinearRotation rotationMoveSecondSample = new LinearRotation(new TimeSpan(splineMoveSamplesTimes[2], splineMoveSamplesTimes[4]),
                new RotationState(Math.toRadians(-129)),
                new RotationState(Math.toRadians(-219))
        );

        LinearElbow elbowUpSecondSample = new LinearElbow(clawCloseSecondSample.getEndTime(),
                new ElbowState(elbowDown),
                new ElbowState(elbowPartiallyUp)
        );

        LinearClaw clawOpenSecondSample = new LinearClaw(rotationMoveSecondSample.getEndTime(),
                new ClawState(clawClosed),
                new ClawState(clawOpen)
        );

        // Pick up third sample
        LinearRotation rotationApproachThirdSample = new LinearRotation(new TimeSpan(splineMoveSamplesTimes[4], splineMoveSamplesTimes[6]),
                new RotationState(Math.toRadians(-219)),
                new RotationState(Math.toRadians(-129))
        );

        LinearElbow elbowDownThirdSample = new LinearElbow(splineMoveSamplesTimes[6]-elbowClipDrive,
                new ElbowState(elbowPartiallyUp),
                new ElbowState(elbowDown)
        );

        LinearClaw clawCloseThirdSample = new LinearClaw(elbowDownThirdSample.getEndTime()-clawClipElbow,
                new ClawState(clawOpen),
                new ClawState(clawClosed)
        );

        // Drop third sample
        LinearRotation rotationMoveThirdSample = new LinearRotation(new TimeSpan(splineMoveSamplesTimes[6], splineMoveSamplesTimes[8]),
                new RotationState(Math.toRadians(-129)),
                new RotationState(Math.toRadians(-270))
        );

        LinearElbow elbowUpThirdSample = new LinearElbow(new TimeSpan(clawCloseThirdSample.getEndTime(), rotationMoveThirdSample.getEndTime()-0.2),
                new ElbowState(elbowDown),
                new ElbowState(elbowPartiallyUp)
        );

        LinearClaw clawOpenThirdSample = new LinearClaw(elbowUpThirdSample.getEndTime(),
                new ClawState(clawClosed),
                new ClawState(clawOpen)
        );



        ////////////////////////////
        // PICK UP FIRST SPECIMEN //
        ////////////////////////////

        LinearElbow elbowDownFirstSpecimen = new LinearElbow(splineMoveSamples.getEndTime()-0.1, // clip end of spline
                new ElbowState(elbowPartiallyUp),
                new ElbowState(elbowDown)
        );

        LinearClaw clawCloseFirstSpecimen = new LinearClaw(elbowDownFirstSpecimen.getEndTime()-0.1, // clip end of elbow
                new ClawState(clawOpen),
                new ClawState(clawClosed)
        );



        //////////////////////////
        // SCORE FIRST SPECIMEN //
        //////////////////////////

        // Approach
        LinearTranslation lineScoreFirstSpecimen = new LinearTranslation(clawCloseFirstSpecimen.getEndTime(),
                new TranslationState(-48, 41),
                new TranslationState(-10, 37)
        );
//        System.out.println(lineScoreFirstSpecimen.getDuration());

        LinearRotation rotationScoreFirstSpecimen = new LinearRotation(new TimeSpan(lineScoreFirstSpecimen.getStartTime(), lineScoreFirstSpecimen.getEndTime()-0.2),
                new RotationState(Math.toRadians(90)),
                new RotationState(Math.toRadians(-90))
        );
//        System.out.println(rotationScoreFirstSpecimen.getDuration());

        LinearLift liftUpFirstSpecimen = new LinearLift(lineScoreFirstSpecimen.getStartTime(),
                new LiftState(-50),
                new LiftState(1350)
        );
//        System.out.println(liftUpFirstSpecimen.getDuration());

        LinearElbow elbowUpFirstSpecimen = new LinearElbow(lineScoreFirstSpecimen.getStartTime(),
                new ElbowState(elbowDown),
                new ElbowState(elbowUp)
        );
//        System.out.println(elbowUpFirstSpecimen.getDuration());

        // Score
        LinearLift liftDownFirstSpecimen = new LinearLift(lineScoreFirstSpecimen.getEndTime()-0.2,
                new LiftState(1350),
                new LiftState(-50)
        );

        LinearClaw clawOpenFirstSpecimen = new LinearClaw(liftDownFirstSpecimen.getStartTime()+.35,
                new ClawState(clawClosed),
                new ClawState(clawOpen)
        );



        /////////////////////////////
        // PICK UP SECOND SPECIMEN //
        /////////////////////////////

        LinearTranslation lineApproachSecondSpecimen = new LinearTranslation(clawOpenFirstSpecimen.getEndTime(),
                new TranslationState(-10, 37),
                new TranslationState(-48, 41)
        );
//        System.out.println("Cycle time: " + (lineApproachSecondSpecimen.getEndTime() - lineScoreFirstSpecimen.getStartTime()));

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




        ///////////////////////////
        // SCORE SECOND SPECIMEN //
        ///////////////////////////

        // Approach
        LinearTranslation lineScoreSecondSpecimen = new LinearTranslation(clawCloseSecondSpecimen.getEndTime(),
                new TranslationState(-48, 41),
                new TranslationState(-10, 37)
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





        ////////////////////////////
        // PICK UP THIRD SPECIMEN //
        ////////////////////////////

        LinearTranslation lineApproachThirdSpecimen = new LinearTranslation(clawOpenSecondSpecimen.getEndTime(),
                new TranslationState(-10, 37),
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





        //////////////////////////
        // SCORE THIRD SPECIMEN //
        //////////////////////////

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





        /////////////////////////////
        // PICK UP FOURTH SPECIMEN //
        /////////////////////////////

        LinearTranslation lineApproachFourthSpecimen = new LinearTranslation(clawOpenThirdSpecimen.getEndTime(),
                new TranslationState(-10, 37),
                new TranslationState(-48, 41)
        );

        LinearRotation rotationApproachFourthSpecimen = new LinearRotation(new TimeSpan(lineApproachFourthSpecimen.getStartTime(), lineApproachFourthSpecimen.getEndTime()-0.2),
                new RotationState(Math.toRadians(-90)),
                new RotationState(Math.toRadians(90))
        );

        LinearElbow elbowDownFourthSpecimen = new LinearElbow(lineApproachFourthSpecimen.getEndTime()-0.8, // clip end of rotation
                new ElbowState(elbowUp),
                new ElbowState(elbowDown)
        );

        LinearClaw clawCloseFourthSpecimen = new LinearClaw(elbowDownFourthSpecimen.getEndTime()-0.1, // clip end of elbow
                new ClawState(clawOpen),
                new ClawState(clawClosed)
        );





        ///////////////////////////
        // SCORE FOURTH SPECIMEN //
        ///////////////////////////

        // Approach
        LinearTranslation lineScoreFourthSpecimen = new LinearTranslation(clawCloseFourthSpecimen.getEndTime(),
                new TranslationState(-48, 41),
                new TranslationState(-10, 37)
        );

        LinearRotation rotationScoreFourthSpecimen = new LinearRotation(new TimeSpan(lineScoreFourthSpecimen.getStartTime(), lineScoreFourthSpecimen.getEndTime()-0.2),
                new RotationState(Math.toRadians(90)),
                new RotationState(Math.toRadians(-90))
        );

        LinearLift liftUpFourthSpecimen = new LinearLift(lineScoreFourthSpecimen.getStartTime(),
                new LiftState(-50),
                new LiftState(1350)
        );

        LinearElbow elbowUpFourthSpecimen = new LinearElbow(lineScoreFourthSpecimen.getStartTime(),
                new ElbowState(elbowDown),
                new ElbowState(elbowUp)
        );

        // Score
        LinearLift liftDownFourthSpecimen = new LinearLift(lineScoreFourthSpecimen.getEndTime()-0.2,
                new LiftState(1350),
                new LiftState(-50)
        );

        LinearClaw clawOpenFourthSpecimen = new LinearClaw(liftDownFourthSpecimen.getStartTime()+.35,
                new ClawState(clawClosed),
                new ClawState(clawOpen)
        );




        //////////////////////////////
        // PARK IN OBSERVATION ZONE //
        //////////////////////////////

        LinearTranslation linePark = new LinearTranslation(clawOpenFourthSpecimen.getEndTime(),
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

        TranslationPlan translationPlan = new TranslationPlan(robot,
                // PRELOAD
                splinePreload,

                // MOVING SAMPLES
                splineApproachSamples,
                splineMoveSamples,

                // SCORING SPECIMENS
                lineScoreFirstSpecimen,

                lineApproachSecondSpecimen,
                lineScoreSecondSpecimen,

                lineApproachThirdSpecimen,
                lineScoreThirdSpecimen,

                lineApproachFourthSpecimen,
                lineScoreFourthSpecimen,

                linePark
        );

        RotationPlan rotationPlan = new RotationPlan(robot,
                // PRELOAD
                rotationStillPreload,

                // MOVING SAMPLES
                rotationApproachSamples,
                rotationMoveFirstSample,
                rotationApproachSecondSample,
                rotationMoveSecondSample,
                rotationApproachThirdSample,
                rotationMoveThirdSample,

                // SCORING SPECIMENS
                rotationScoreFirstSpecimen,

                rotationApproachSecondSpecimen,
                rotationScoreSecondSpecimen,

                rotationApproachThirdSpecimen,
                rotationScoreThirdSpecimen,

                rotationApproachFourthSpecimen,
                rotationScoreFourthSpecimen
        );

        LiftPlan liftPlan = new LiftPlan(robot,
                // PRELOAD
                liftUpPreload,
                liftDownPreload,

                // SCORING SPECIMENS
                liftUpFirstSpecimen,
                liftDownFirstSpecimen,

                liftUpSecondSpecimen,
                liftDownSecondSpecimen,

                liftUpThirdSpecimen,
                liftDownThirdSpecimen,

                liftUpFourthSpecimen,
                liftDownFourthSpecimen
        );


        ClawPlan clawPlan = new ClawPlan(robot,
                // PRELOAD
                clawOpenPreload,

                // MOVING SAMPLES
                clawCloseFirstSample,
                clawOpenFirstSample,
                clawCloseSecondSample,
                clawOpenSecondSample,
                clawCloseThirdSample,
                clawOpenThirdSample,

                // SCORING SPECIMENS
                clawCloseFirstSpecimen,
                clawOpenFirstSpecimen,

                clawCloseSecondSpecimen,
                clawOpenSecondSpecimen,

                clawCloseThirdSpecimen,
                clawOpenThirdSpecimen,

                clawCloseFourthSpecimen,
                clawOpenFourthSpecimen,

                clawPark
        );


        ElbowPlan elbowPlan = new ElbowPlan(robot,
                // PRELOAD
                elbowStillPreload,

                // MOVING SAMPLES
                elbowApproachSamples,

                elbowDownFirstSample,
                elbowUpFirstSample,
                elbowDownSecondSample,
                elbowUpSecondSample,
                elbowDownThirdSample,
                elbowUpThirdSample,

                // SCORING SPECIMENS
                elbowDownFirstSpecimen,
                elbowUpFirstSpecimen,

                elbowDownSecondSpecimen,
                elbowUpSecondSpecimen,

                elbowDownThirdSpecimen,
                elbowUpThirdSpecimen,

                elbowDownFourthSpecimen,
                elbowUpFourthSpecimen,

                elbowPark
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