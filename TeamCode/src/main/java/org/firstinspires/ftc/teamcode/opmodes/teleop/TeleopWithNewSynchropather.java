package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotSystem;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.Command;

@TeleOp(name = "sigma")
//IMPORTANT: this is for the old robot. I couldnt really merge the four other branches of krakens code into this.
public class TeleopWithNewSynchropather extends LinearOpMode {
    //up, down, open, closed positions are defined in command class since action happens there
    public Command command;
    public RobotSystem robot;
    @Override
    public void runOpMode() {
        //defining robot in both this class and command class, passing through constructor to command class
        //this allows for command class to access robot in this class.
        robot = new RobotSystem(hardwareMap, new Pose2d(0, 0, new Rotation2d(Math.toRadians(-90))), false, this);
        command = new Command(robot);
        //Command.resetCommand(); not really necessary
        robot.logOdometry();
        robot.localization.update();
        waitForStart();
    while (opModeIsActive()) {
        double defaultSpeed = 0.5;
        //regular field centric drive.
        double strafe = -gamepad1.left_stick_x * defaultSpeed;
        double turn = gamepad1.right_stick_x * defaultSpeed;
        double forward = gamepad1.left_stick_y * defaultSpeed;
        robot.drive.driveFieldCentricPowers(strafe, forward, turn, Math.toDegrees(robot.localization.getH()));
        //example implementation
        if(gamepad1.a) {
            //add actions to 5-line string, populating the commandlist array
            //evaluate the command, starting and cycling through words with switch loops
            //automatically resets after each eval
            Command.Add("EU", "null", "null", "ull", "null");
            command.evalCommand();
        }
        //this system excludes lift. however, it would be possible to implement lift as well.
    }
    }
}
