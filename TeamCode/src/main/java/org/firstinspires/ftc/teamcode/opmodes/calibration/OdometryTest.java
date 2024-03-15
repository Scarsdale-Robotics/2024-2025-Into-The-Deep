package org.firstinspires.ftc.teamcode.opmodes.calibration;

import com.arcrobotics.ftclib.purepursuit.Path;
import com.arcrobotics.ftclib.purepursuit.Waypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


public class OdometryTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Path path = new Path();
        Waypoint waypoint = new StartWaypoint();
    }

//    @Override
//    public void loop() {
//        odometry.update();
//        Pose2d currentPose = odometry.getPose();
//        // Use currentPose for navigation or telemetry
//    }

}
