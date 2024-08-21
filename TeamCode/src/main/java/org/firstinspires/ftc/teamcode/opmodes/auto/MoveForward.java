package org.firstinspires.ftc.teamcode.opmodes.auto;

// Imports (don't worry about these)
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.HardwareRobot;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

// Name your program here (it will show up as an option when we run it on the robot's Driver Hub)
@Autonomous(name="Definitely not hazardous code :)")
public class MoveForward extends LinearOpMode {

    // This override exists because we're inheriting all the methods from LinearOpMode
    // and writing our own implementation for this function, runOpMode().
    @Override
    public void runOpMode() {
        // We initialize the software here before Start is pressed.
        // Create a HardwareRobot object, which holds all of the robot's Motor, Servo, etc. objects.
        HardwareRobot robot = new HardwareRobot(hardwareMap);
        // Create a DriveSubsystem object, which provides an easy way to drive the robot.
        DriveSubsystem drive = new DriveSubsystem(
                robot.leftFront,
                robot.rightFront,
                robot.leftBack,
                robot.rightBack,
                robot.imu,
                this,
                telemetry
        );

        // wait for Start button press...
        waitForStart();

        // Drive forward forever!!!!!!!!!
        // For while loops, you NEED to put opModeIsActive() as the condition, or else the robot
        // won't stop when stopped!
        while (opModeIsActive()) {
            drive.driveRobotCentric(0, 0.5, 0);

            // driveRobotCentric means that the robot will drive according to its own perspective,
            // e.g. driving forward means the robot will head straight in its current direction.

            // Another thing, these values can only range between 0 and 1, with 1 representing full
            // power and 0 representing no power.
        }

    }

}
