package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp(name="Red TeleOp")
public class RedTeleop extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        TeleopUtil teleop = new TeleopUtil(true, this);

        waitForStart();

        while (opModeIsActive()) {
            teleop.tick();
        }
    }

}
