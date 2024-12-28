package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotSystem;
import org.firstinspires.ftc.teamcode.subsystems.indep.InDepSubsystem;

public class Teleop extends LinearOpMode {
    private RobotSystem robot;
    private InDepSubsystem indep;

    private boolean manualIndepLaw = false;
    private boolean lastManualIndepControlPressed = false;

    /*
    WARNINGS
    - 500ms RUMBLE: MANUAL INDEP LAW ACTIVATED
        - 100ms RUMBLE: MANUAL INDEP LAW DEACTIVATED
     */

    @Override
    public void runOpMode() throws InterruptedException {
        this.robot = new RobotSystem(hardwareMap, new Pose2d(0, 0, new Rotation2d(Math.toRadians(-90))), false, this);

        this.indep = robot.inDep;

        waitForStart();

        while (opModeIsActive()) {
            boolean manualIndepControlPressed =
                    (gamepad1.dpad_left && gamepad1.cross) ||
                    (gamepad2.dpad_left && gamepad2.cross);
            if (
                lastManualIndepControlPressed && !manualIndepControlPressed
            ) {
                manualIndepLaw = !manualIndepLaw;

                int rumbleTimeMS = 100;
                if (manualIndepLaw) rumbleTimeMS = 500;  // if manual law activated, buzz 500ms

                gamepad1.rumble(rumbleTimeMS);
                gamepad2.rumble(rumbleTimeMS);
            }
            lastManualIndepControlPressed = manualIndepControlPressed;

            // TODO: INDEP UNDOs
            if (manualIndepLaw) {
                // write manual controls
            } else {
                indep.tick(
                        gamepad1.triangle,
                        gamepad2.triangle,
                        gamepad2.square,
                        gamepad1.circle,
                        gamepad1.square
                );
            }
        }
    }

}
