package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotSystem;
@TeleOp(name = "sigma")
//IMPORTANT - I was being lazy and didnt want to merge with the 4 other branches that have Krakens code
//so, this is for the old robot. sry if that creates any problems
public class TeleopWithNewSynchropather extends LinearOpMode {
    private RobotSystem robot;
    public void runOpMode() {
        /**regular initalization...i also don't know what the difference
         *    between robotsystem and autonomousrobot is so ill use robotsys for now
         */
        this.robot = new RobotSystem(hardwareMap, new Pose2d(0, 0, new Rotation2d(Math.toRadians(-90))), false, this);
        robot.logOdometry();
        robot.localization.update();
        waitForStart();
        double speed = 1;
    while (opModeIsActive()) {
        //regular field centric drive, doesnt really matter
        //i also keep forgetting strafe is inverted smh
        double strafe = -gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;
        double forward = gamepad1.left_stick_y;
        robot.drive.driveFieldCentricPowers(strafe, forward, turn, Math.toDegrees(robot.localization.getH()));
        //also confused about fieldcentric vs fieldcentricpowers....
    }











    }
}
