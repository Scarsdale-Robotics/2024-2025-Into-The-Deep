package org.firstinspires.ftc.teamcode.opmodes.calibration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.RobotSystem;
import org.firstinspires.ftc.teamcode.subsystems.GoBildaPinpointDriver;

import java.util.Locale;

@TeleOp(name="Pinpoint Logger", group="Calibration")
public class PinpointLogger extends LinearOpMode {

    GoBildaPinpointDriver pinpoint;
    double oldTime = 0;


    private static final double xOffset = 144.0, yOffset = -120.0;
    private static final double ENCODER_CPR = 4096; // Optii v1
    private static final double WHEEL_DIAMETER = 35.0; // mm
    private static final double TICKS_PER_MM = ENCODER_CPR / (Math.PI * WHEEL_DIAMETER);

    @Override
    public void runOpMode() throws InterruptedException {
        this.telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");

        // As of 12/1/24
        // X wheel on the left, Y wheel on the right
        pinpoint.setOffsets(xOffset, yOffset);

        // Optii v1
        telemetry.addData("Encoder res", TICKS_PER_MM);
        pinpoint.setEncoderResolution(TICKS_PER_MM);

        /*
        Set the direction that each of the two odometry pods count. The X (forward) pod should
        increase when you move the robot forward. And the Y (strafe) pod should increase when
        you move the robot to the left.
         */
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        pinpoint.resetPosAndIMU();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", pinpoint.getXOffset());
        telemetry.addData("Y offset", pinpoint.getYOffset());
        telemetry.addData("Device Version Number:", pinpoint.getDeviceVersion());
        telemetry.addData("Device Scalar", pinpoint.getYawScalar());
        telemetry.update();

        waitForStart();
        resetRuntime();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            /*
            Request an update from the Pinpoint odometry computer. This checks almost all outputs
            from the device in a single I2C read.
             */
            pinpoint.update();

            /*
            Optionally, you can update only the heading of the device. This takes less time to read, but will not
            pull any other data. Only the heading (which you can pull with getHeading() or in getPosition().
             */
            //odo.update(GoBildaPinpointDriver.readData.ONLY_UPDATE_HEADING);


            if (gamepad1.square){
                pinpoint.resetPosAndIMU(); //resets the position to 0 and recalibrates the IMU
            }

            if (gamepad1.cross){
                pinpoint.recalibrateIMU(); //recalibrates the IMU without resetting position
            }

            /*
            This code prints the loop frequency of the REV Control Hub. This frequency is effected
            by I²C reads/writes. So it's good to keep an eye on. This code calculates the amount
            of time each cycle takes and finds the frequency (number of updates per second) from
            that cycle time.
             */
            double newTime = getRuntime();
            double loopTime = newTime-oldTime;
            double frequency = 1/loopTime;
            oldTime = newTime;


            /*
            gets the current Position (x & y in mm, and heading in degrees) of the robot, and prints it.
             */
            Pose2D pos = pinpoint.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);

            // draw current pose
            Pose2d currentPose = new Pose2d(pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH), new Rotation2d(pos.getHeading(AngleUnit.RADIANS)));
            TelemetryPacket posePacket = new TelemetryPacket();
            posePacket.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(posePacket.fieldOverlay(), currentPose);
            FtcDashboard.getInstance().sendTelemetryPacket(posePacket);

            /*
            gets the current Velocity (x & y in mm/sec and heading in degrees/sec) and prints it.
             */
            Pose2D vel = pinpoint.getVelocity();
            String velocity = String.format(Locale.US,"{XVel: %.3f, YVel: %.3f, HVel: %.3f}", vel.getX(DistanceUnit.MM), vel.getY(DistanceUnit.MM), vel.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Velocity", velocity);

            // draw current pose
            Pose2d currentVelocity = new Pose2d(vel.getX(DistanceUnit.INCH), vel.getY(DistanceUnit.INCH), new Rotation2d(vel.getHeading(AngleUnit.RADIANS)));
            TelemetryPacket velocityPacket = new TelemetryPacket();
            Drawing.drawVelocity(posePacket.fieldOverlay(), currentPose, currentVelocity);
            FtcDashboard.getInstance().sendTelemetryPacket(velocityPacket);

            /*
            Gets the Pinpoint device status. Pinpoint can reflect a few states. But we'll primarily see
            READY: the device is working as normal
            CALIBRATING: the device is calibrating and outputs are put on hold
            NOT_READY: the device is resetting from scratch. This should only happen after a power-cycle
            FAULT_NO_PODS_DETECTED - the device does not detect any pods plugged in
            FAULT_X_POD_NOT_DETECTED - The device does not detect an X pod plugged in
            FAULT_Y_POD_NOT_DETECTED - The device does not detect a Y pod plugged in
            */
            telemetry.addData("Status", pinpoint.getDeviceStatus());

            telemetry.addData("Pinpoint Frequency", pinpoint.getFrequency()); //prints/gets the current refresh rate of the Pinpoint

            telemetry.addData("REV Hub Frequency: ", frequency); //prints the control system refresh rate


            telemetry.addData("X ticks", pinpoint.getEncoderX());
            telemetry.addData("Y ticks", pinpoint.getEncoderY());
            telemetry.update();

        }

    }

}
