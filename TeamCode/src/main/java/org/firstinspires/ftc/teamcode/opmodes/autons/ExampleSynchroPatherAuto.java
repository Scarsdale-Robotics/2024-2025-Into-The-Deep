package org.firstinspires.ftc.teamcode.opmodes.autons;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotSystem;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.Synchronizer;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.TimeSpan;
import org.firstinspires.ftc.teamcode.synchropather.systems.rotation.RotationPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.rotation.RotationState;
import org.firstinspires.ftc.teamcode.synchropather.systems.rotation.movements.LinearRotation;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.TranslationPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.TranslationState;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.movements.CRSplineTranslation;
import org.firstinspires.ftc.teamcode.synchropather.systems.translation.movements.LinearTranslation;

@Autonomous(name="Example SynchroPather Auto")
public class ExampleSynchroPatherAuto extends LinearOpMode {

    RobotSystem robot;
    Synchronizer synchronizer;

    @Override
    public void runOpMode() throws InterruptedException {
        this.robot = new RobotSystem(hardwareMap, new Pose2d(), this);
        initSynchronizer();

        waitForStart();

        synchronizer.start();
        while (opModeIsActive() && !synchronizer.update());
        synchronizer.stop();
    }


    private void initSynchronizer() {
        // Translation plan
        LinearTranslation line1 = new LinearTranslation(0,
                new TranslationState(0, 0),
                new TranslationState(24, 0)
        );
        LinearTranslation line2 = new LinearTranslation(0,
                new TranslationState(24, 0),
                new TranslationState(0, 0)
        );
        TranslationPlan translationPlan = new TranslationPlan(robot,
                line1,
                line2
        );

        // Rotation plan
        LinearRotation rotation = new LinearRotation(new TimeSpan(0, line2.getEndTime()),
                new RotationState(Math.toRadians(0)),
                new RotationState(Math.toRadians(360))
        );
        RotationPlan rotationPlan = new RotationPlan(robot,
                rotation
        );

        // Synchronizer
        this.synchronizer = new Synchronizer(
                translationPlan,
                rotationPlan
        );
    }

}
