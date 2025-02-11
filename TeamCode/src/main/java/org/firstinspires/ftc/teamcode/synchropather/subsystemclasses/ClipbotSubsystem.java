package org.firstinspires.ftc.teamcode.synchropather.subsystemclasses;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.outoftheboxrobotics.photoncore.Photon;
import com.outoftheboxrobotics.photoncore.hardware.motor.PhotonDcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.synchropather.systems.magazine.MagazineConstants;

@Photon
public class ClipbotSubsystem {

    private final Motor magazine;
    private final PhotonDcMotor magazineCurrentSensor;

    private final Servo clipper;

    public final Telemetry telemetry;


    public ClipbotSubsystem(
            Motor magazine,
            Servo clipper,
            Telemetry telemetry
    ) {
        this.magazine = magazine;
        this.magazineCurrentSensor = (PhotonDcMotor) magazine.motor;

        this.clipper = clipper;
        this.telemetry = telemetry;
    }

    /**
     * Runs a magazine homing sequence to detect the end of the magazine.
     */
    public void homeMagazine(LinearOpMode opMode, Telemetry telemetry) {
        // threshold to detect an end stop, in milliamps
        double currentThreshold = 1000;
        double motorCurrent;

        // home and set zero position
        magazine.motor.setPower(0.5);
        motorCurrent = 0;
        while (opMode.opModeIsActive() && motorCurrent < currentThreshold) {
            motorCurrent = Math.abs(getMagazineCurrent());
            telemetry.addData("motorCurrent", motorCurrent);
            telemetry.update();
        }
        magazine.motor.setPower(0);
        MagazineConstants.ZERO_HOME = getMagazinePosition() + MagazineConstants.ZERO_HOME;
    }

    /**
     * @return the magazine motor's current in milliamps
     */
    public double getMagazineCurrent() {
        return magazineCurrentSensor.getCorrectedCurrent(CurrentUnit.MILLIAMPS);
    }

    /**
     * @return the magazine motor's position in inches
     */
    public double getMagazinePosition() {
        return MagazineConstants.ticksToInches(magazine.getCurrentPosition()) - MagazineConstants.ZERO_HOME;
    }

    public void setMagazinePower(double power) {
        magazine.motor.setPower(power);
    }

    public void stopMagazine() {
        magazine.motor.setPower(0);
    }

}
