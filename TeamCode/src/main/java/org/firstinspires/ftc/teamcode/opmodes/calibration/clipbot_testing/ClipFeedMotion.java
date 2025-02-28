package org.firstinspires.ftc.teamcode.opmodes.calibration.clipbot_testing;

import static org.firstinspires.ftc.teamcode.Auxiliary.initMotor;
import static org.firstinspires.ftc.teamcode.Auxiliary.initServo;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.synchropather.subsystemclasses.ClipbotSubsystem;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.Synchronizer;
import org.firstinspires.ftc.teamcode.synchropather.systems.mFeeder.MFeederConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.mFeeder.MFeederPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.mFeeder.MFeederState;
import org.firstinspires.ftc.teamcode.synchropather.systems.mFeeder.movements.LinearMFeeder;
import org.firstinspires.ftc.teamcode.synchropather.systems.mIntake.MIntakeConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.mLoader.MLoaderConstants;

import java.util.ArrayDeque;

@Autonomous(name="Clip Feed Motion", group = "Calibration")
public class ClipFeedMotion extends LinearOpMode {

    private Synchronizer synchronizer;

    private ArrayDeque<Double> loopTicks;
    private ElapsedTime runtime;

    private Servo magazineIntake;
    private Servo magazineLoader1;
    private Servo magazineLoader2;
    private Motor magazineFeeder;

    private ClipbotSubsystem clipbot;

    private int clipInventory = MFeederConstants.RELOAD_CAPACITY;
    private boolean inventoryStocked = true;

    @Override
    public void runOpMode() throws InterruptedException {
        initSubsystems();

        loopTicks = new ArrayDeque<>();
        runtime = new ElapsedTime(0);
        runtime.reset();

        telemetry.addData("[MAIN] TPS", 0);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Wait for button press
            while (opModeIsActive() && (gamepad1.triangle || !inventoryStocked)) {
                updateTPS();
                if (gamepad1.square) {
                    inventoryStocked = true;
                    clipInventory = MFeederConstants.RELOAD_CAPACITY;
                }
            }
            while (opModeIsActive() && !gamepad1.triangle) updateTPS();
            if (clipInventory <= 0) continue;
            initAdvanceMagazineMotion();

            // pick up
            synchronizer.start();
            while (opModeIsActive() && synchronizer.update()) {
                updateTPS();
            }
            while (opModeIsActive() && !gamepad1.triangle) {
                updateTPS();
                synchronizer.update();
            }
            synchronizer.stop();
            updateTPS();
        }
    }

    private void initSubsystems() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        magazineIntake = hardwareMap.get(ServoImplEx.class, "magazineIntake");
        magazineLoader1 = hardwareMap.get(ServoImplEx.class, "magazineLoaderClose");
        magazineLoader2 = hardwareMap.get(ServoImplEx.class, "magazineLoaderFar");

        magazineFeeder = new MotorEx(hardwareMap, "magazineFeeder", Motor.GoBILDA.RPM_1620);
        magazineFeeder.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        magazineFeeder.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        magazineFeeder.setRunMode(Motor.RunMode.RawPower);
        magazineFeeder.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        magazineFeeder.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        magazineFeeder.setInverted(false);

        this.clipbot = new ClipbotSubsystem(
                magazineIntake,
                magazineLoader1,
                magazineLoader2,
                initServo(hardwareMap, "klipper"),
                initMotor(hardwareMap, "magazineFeeder", Motor.GoBILDA.RPM_1620),
                telemetry
        );
    }

    private void updateTPS() {
        // TPS counter
        double currentTime = runtime.seconds();
        loopTicks.add(currentTime);
        while (!loopTicks.isEmpty() && currentTime - loopTicks.getFirst() > 1d) loopTicks.removeFirst();
        telemetry.addData("[MAIN] TPS", loopTicks.size());
        telemetry.addData("clipInventory", clipInventory);
        clipbot.update();
        telemetry.update();
    }

    private void initAdvanceMagazineMotion() {
        if (!inventoryStocked) return;
        if (clipInventory <= 0) {
            inventoryStocked = false;
            return;
        }

        // Get target states
        double maxClips = MFeederConstants.MAX_CAPACITY;
        MFeederState currentFeederPosition = new MFeederState(
                (maxClips - clipInventory) * MFeederConstants.INCHES_PER_CLIP
        );

        clipInventory--;
        MFeederState targetFeederPosition = new MFeederState(
                (maxClips - clipInventory) * MFeederConstants.INCHES_PER_CLIP
        );


        // Movements
        LinearMFeeder advanceFeeder = new LinearMFeeder(0,
                currentFeederPosition,
                targetFeederPosition
        );



        //// Plans
        MFeederPlan mFeederPlan;
        if (clipInventory==0) {
            telemetry.addData("Clip Inventory Is Zero!"," TRUE");
            LinearMFeeder resetFeeder = new LinearMFeeder(advanceFeeder.getEndTime(),
                    targetFeederPosition,
                    new MFeederState(0)
            );
            mFeederPlan = new MFeederPlan(clipbot,
                    advanceFeeder,
                    resetFeeder
            );
            inventoryStocked = false;
        } else {
            telemetry.addData("Clip Inventory Is Zero!"," FALSE");
            mFeederPlan = new MFeederPlan(clipbot,
                    advanceFeeder
            );
        }

        telemetry.update();


        // Synchronizer
        this.synchronizer = new Synchronizer(
                mFeederPlan
        );
    }

}
