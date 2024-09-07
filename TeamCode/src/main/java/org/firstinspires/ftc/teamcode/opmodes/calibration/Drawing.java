package org.firstinspires.ftc.teamcode.opmodes.calibration;

///////////////////////////////////
// CREDIT: ROADRUNNER QUICKSTART //
///////////////////////////////////

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

public final class Drawing {
    private Drawing() {}


    public static void drawRobot(Canvas c, com.arcrobotics.ftclib.geometry.Pose2d t) {
        Pose2d t_rr = new Pose2d(t.getX(), t.getY(), t.getHeading());
        final double ROBOT_RADIUS = 9;

        c.setStrokeWidth(1);
        c.setStroke("#3f51b5");
        c.strokeCircle(t_rr.position.x, t_rr.position.y, ROBOT_RADIUS);

        Vector2d halfv = t_rr.heading.vec().times(0.5 * ROBOT_RADIUS);
        Vector2d p1 = t_rr.position.plus(halfv);
        Vector2d p2 = p1.plus(halfv);
        c.strokeLine(p1.x, p1.y, p2.x, p2.y);
    }


    public static void drawTargetPose(Canvas c, com.arcrobotics.ftclib.geometry.Pose2d t) {
        Pose2d t_rr = new Pose2d(t.getX(), t.getY(), t.getHeading());
        final double ROBOT_RADIUS = 9;

        c.setStrokeWidth(1);
        c.setStroke("#3fb543");
        c.strokeCircle(t_rr.position.x, t_rr.position.y, ROBOT_RADIUS);

        Vector2d halfv = t_rr.heading.vec().times(0.5 * ROBOT_RADIUS);
        Vector2d p1 = t_rr.position.plus(halfv);
        Vector2d p2 = p1.plus(halfv);
        c.strokeLine(p1.x, p1.y, p2.x, p2.y);
    }
}