package org.firstinspires.ftc.teamcode.synchropather.subsystemclasses;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.outoftheboxrobotics.photoncore.hardware.motor.PhotonDcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.synchropather.systems.mFeeder.MFeederConstants;

public class ClipbotSubsystem {

    private final Servo magazineIntake;
    private final Servo magazineLoader;

    private final Motor magazineFeeder;
    private final PhotonDcMotor magazineFeederCurrentSensor;


    public ClipbotSubsystem(
            Servo magazineIntake,
            Servo magazineLoader,
            Motor magazineFeeder
    ) {

        // Servo that takes clips off the wall
        this.magazineIntake = magazineIntake;

        // Servo that snaps the clips from the intake onto the lead screw rail
        this.magazineLoader = magazineLoader;

        // Motor that controls the lead screw position
        this.magazineFeeder = magazineFeeder;
        this.magazineFeederCurrentSensor = (PhotonDcMotor) magazineFeeder.motor;

    }

    public void setMagazineIntakePosition(double servoPosition) {
        magazineIntake.setPosition(servoPosition);
    }

    public void setMagazineLoaderPosition(double servoPosition) {
        magazineLoader.setPosition(servoPosition);
    }

    /**
     * Runs a magazine feeder homing sequence to detect the end of the magazine.
     */
    public void homeMagazineFeeder(LinearOpMode opMode, Telemetry telemetry) {
        // threshold to detect an end stop, in milliamps
        double currentThreshold = 1000;
        double motorCurrent;

        // home and set zero position
        magazineFeeder.motor.setPower(0.5);
        motorCurrent = 0;
        while (opMode.opModeIsActive() && motorCurrent < currentThreshold) {
            motorCurrent = Math.abs(getMagazineFeederCurrent());
            telemetry.addData("motorCurrent", motorCurrent);
            telemetry.update();
        }
        magazineFeeder.motor.setPower(0);
        MFeederConstants.ZERO_HOME = magazineFeeder.getCurrentPosition();
    }

    /**
     * @return the magazine feeder motor's current in milliamps
     */
    public double getMagazineFeederCurrent() {
        return magazineFeederCurrentSensor.getCorrectedCurrent(CurrentUnit.MILLIAMPS);
    }

    /**
     * @return the magazine feeder's position in inches
     */
    public double getMagazineFeederPosition() {
        return MFeederConstants.ticksToInches(magazineFeeder.getCurrentPosition()) - MFeederConstants.ZERO_HOME;
    }

    public void setMagazineFeederPower(double power) {
        magazineFeeder.motor.setPower(power);
    }

    public void stopMagazineFeeder() {
        magazineFeeder.motor.setPower(0);
    }

}
