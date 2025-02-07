package org.firstinspires.ftc.teamcode.opmodes.calibration;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.Synchronizer;
import org.firstinspires.ftc.teamcode.synchropather.systems.extendo.ExtendoPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.extendo.ExtendoState;
import org.firstinspires.ftc.teamcode.synchropather.systems.extendo.movements.LinearExtendo;

public class haiwfhuaiwe extends LinearOpMode {

    private Synchronizer synchronizer;
    @Override
    public void runOpMode() throws InterruptedException {
        initSynchronizer();

        waitForStart();

        boolean synchronizerRunning = false;
        while (opModeIsActive()) {
            if (gamepad1.square) {
                synchronizer.start();
                synchronizerRunning = true;
            }

            if (synchronizerRunning) {
                synchronizerRunning = synchronizer.update();
            }
        }
    }

    private void initSynchronizer(){
        LinearExtendo extendo1 = new LinearExtendo(0,
            new ExtendoState(0),
                new ExtendoState(12)
        );
        ExtendoPlan extendoPlan = new ExtendoPlan(null,
                extendo1
        );
        this.synchronizer = new Synchronizer(extendoPlan);
    }
}
