package org.firstinspires.ftc.teamcode.opmodes.autons;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareRobot;
import org.firstinspires.ftc.teamcode.RobotSystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.Synchronizer;
import org.firstinspires.ftc.teamcode.synchropather.systems.rotation.LinearRotation;
import org.firstinspires.ftc.teamcode.synchropather.systems.rotation.RotationPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.rotation.RotationState;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.CRSplineTranslation;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.LinearTranslation;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.TranslationPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.TranslationState;

@Autonomous(name="Kenneth Auto")
public class KennethAuto extends LinearOpMode {

    public double posX = 0;
    public double posY = 0;

    // final private ElapsedTime runtime = new ElapsedTime();
    // private DriveSubsystem drive;
    public static String sequence = "";
    public static boolean useString = false;


    @Override
    public void runOpMode() throws InterruptedException {

//        RobotSystem robot = new RobotSystem(hardwareMap, false, this, telemetry);
//        robot.getInDep().autoInit();
//        robot.getCV().autoExposure();
//        drive = robot.getDrive();
//        runtime.reset();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        RobotSystem robot = new RobotSystem(hardwareMap, this);



        waitForStart();

        // translation Plan
        CRSplineTranslation spline1 = new CRSplineTranslation(0,
                new TranslationState(-40.75,63.5),
                new TranslationState(-40.75,38),
                new TranslationState(43,36)
        );

        CRSplineTranslation spline2 = new CRSplineTranslation(spline1.getEndTime(),
                new TranslationState(43,36),
                new TranslationState(0,12),
                new TranslationState(-56,12)
        );

        CRSplineTranslation spline3 = new CRSplineTranslation(spline2.getEndTime(),
                new TranslationState(-56,12),
                new TranslationState(12,12),
                new TranslationState(43,36)
        );

        CRSplineTranslation spline4 = new CRSplineTranslation(spline3.getEndTime(),
                new TranslationState(43,36),
                new TranslationState(0,12),
                new TranslationState(-36,12),
                new TranslationState(-56,24)
        );

        CRSplineTranslation spline5 = new CRSplineTranslation(spline4.getEndTime(),
                new TranslationState(-56,24),
                new TranslationState(-36,12),
                new TranslationState(0,12),
                new TranslationState(43,36)
        );

        LinearTranslation line1 = new LinearTranslation(spline5.getEndTime(),
                new TranslationState(43,36),
                new TranslationState(43,12)
        );

        LinearTranslation line2 = new LinearTranslation(line1.getEndTime(),
                new TranslationState(43,12),
                new TranslationState(53,12)
        );

        LinearTranslation line3 = new LinearTranslation(line2.getEndTime(),
                new TranslationState(53,12),
                new TranslationState(0,0)
        );

        LinearTranslation line4 = new LinearTranslation(line3.getEndTime()+3,
                new TranslationState(0, 0),
                new TranslationState(48, 60)
        );

        LinearTranslation line5 = new LinearTranslation(line4.getEndTime(),
                new TranslationState(48, 60),
                new TranslationState(-48, 60)
        );

        LinearTranslation line6 = new LinearTranslation(line5.getEndTime(),
                new TranslationState(-48, 60),
                new TranslationState(-48, -60)
        );

        LinearTranslation line7 = new LinearTranslation(line6.getEndTime(),
                new TranslationState(-48, -60),
                new TranslationState(48, -60)
        );

        LinearTranslation line8 = new LinearTranslation(line7.getEndTime(),
                new TranslationState(48, -60),
                new TranslationState(0,0)
        );

        CRSplineTranslation returnToStart = new CRSplineTranslation(line8.getEndTime(),
                new TranslationState(0,0),
                new TranslationState(-36,12),
                new TranslationState(-40.75,63.5)
        );

        TranslationPlan translationPlan = new TranslationPlan(robot,
                spline1,
                spline2,
                spline3,
                spline4,
                spline5,
                line1,
                line2,
                line3,
                line4,
                line5,
                line6,
                line7,
                line8,
                returnToStart
        );


        // rotation Plan
        LinearRotation rot1 = new LinearRotation(0,
                new RotationState(0),
                new RotationState(Math.toRadians(360))
        );

        RotationPlan rotationPlan = new RotationPlan(robot,
                rot1
        );


        // put all the Plans into a Synchronizer
        Synchronizer synchronizer = new Synchronizer(
                translationPlan,
                rotationPlan // initializes the plans here
        );

        // put the MovementSequence into a visualizer object, with timeFactor between 0 and 1 representing the speed of the visualizer
        double timeFactor = 3;
//        Visualizer visualizer = new Visualizer(synchronizer, timeFactor);

        // start visualizer
//        visualizer.start();

        // main visualizer loop with an example telemetry function
//        double targetFPS = 144;
//        while (visualizer.loop()) {
////			generateTelemetry(visualizer, timeFactor);
//            Thread.sleep((int)(1000/targetFPS));
//        }

        double speed = 0.15;
        ElapsedTime runtime = new ElapsedTime(0);
        while (opModeIsActive()) {

            synchronizer.setTarget(runtime.seconds());
            synchronizer.loop();

            // get Position
            robot.localization.getPose();


        }
    }
}
