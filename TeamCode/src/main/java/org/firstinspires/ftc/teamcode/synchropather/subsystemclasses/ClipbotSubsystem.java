package org.firstinspires.ftc.teamcode.synchropather.subsystemclasses;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.outoftheboxrobotics.photoncore.Photon;
import com.outoftheboxrobotics.photoncore.hardware.motor.PhotonDcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.synchropather.systems.mFeeder.MFeederConstants;

public class ClipbotSubsystem {

    private final Servo magazineIntake;
    private final Servo magazineLoader1;
    private final Servo magazineLoader2;
    private final Servo klipper;

    private final Motor magazineFeeder;

    private double magazineFeederPosition;
    private double lastMagazineFeederPosition = Double.MIN_VALUE;

    public final Telemetry telemetry;
    public LinearOpMode opMode;


    public ClipbotSubsystem(
            Servo magazineIntake,
            Servo magazineLoader1,
            Servo magazineLoader2,
            Servo klipper,
            Motor magazineFeeder,
            Telemetry telemetry,
            LinearOpMode opMode
    ) {
        this.telemetry = telemetry;
        this.opMode = opMode;

        // Servo that takes clips off the wall
        this.magazineIntake = magazineIntake;

        // Servo that snaps the clips from the intake onto the lead screw rail
        this.magazineLoader1 = magazineLoader1;
        this.magazineLoader2 = magazineLoader2;

        // Servo that snaps the clips on samples to form specimens
        this.klipper = klipper;

        // Motor that controls the lead screw position
        this.magazineFeeder = magazineFeeder;
        update();

    }

    /**
     * Performs a bulk hardware call and gets feeder motor position and current.
     */
    public void update() {
        if (magazineFeeder!=null && opMode!=null) {
            double obtainedPosition = magazineFeeder.getCurrentPosition();
            if ((obtainedPosition!=0 || lastMagazineFeederPosition==0) && !opMode.isStopRequested()) {
                magazineFeederPosition = obtainedPosition;
            }
            lastMagazineFeederPosition = obtainedPosition;
        }
    }

    public void setMagazineIntakePosition(double servoPosition) {
        magazineIntake.setPosition(servoPosition);
    }

    public void setMagazineLoaderPosition(double servoPosition) {
        magazineLoader1.setPosition(servoPosition);
        magazineLoader2.setPosition(1-servoPosition);
    }

    public void setKlipperPosition(double servoPosition) {
        klipper.setPosition(servoPosition);
    }

    /**
     * @return the magazine feeder's position in inches
     */
    public double getMagazineFeederPosition() {
        return MFeederConstants.ticksToInches(magazineFeederPosition) - MFeederConstants.ZERO_HOME;
    }

    public void setMagazineFeederPower(double power) {
        magazineFeeder.motor.setPower(power);
    }

    /**
     * Sets the magazine feeder motor's power to zero.
     */
    public void stopMagazineFeeder() {
        setMagazineFeederPower(0);
    }

}
