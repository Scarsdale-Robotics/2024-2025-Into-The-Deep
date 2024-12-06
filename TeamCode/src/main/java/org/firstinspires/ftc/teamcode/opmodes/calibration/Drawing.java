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
//        Pose2d t_rr = new Pose2d(t.getY(), -t.getX(), t.getHeading());
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

    public static void drawVelocity(Canvas c, com.arcrobotics.ftclib.geometry.Pose2d robot, com.arcrobotics.ftclib.geometry.Pose2d velocity) {
//        Pose2d t_rr = new Pose2d(t.getY(), -t.getX(), t.getHeading());
        Pose2d robot_rr = new Pose2d(robot.getX(), robot.getY(), robot.getHeading());
        Pose2d velocityEndpoint_rr = new Pose2d(
                robot.getX() + velocity.getX(),
                robot.getY() + velocity.getY(),
                robot.getHeading() + velocity.getHeading()
        );

        c.setStrokeWidth(1);
        c.setStroke("#FF7377");

        Vector2d p1 = robot_rr.position;
        Vector2d p2 = velocityEndpoint_rr.position;
        c.strokeLine(p1.x, p1.y, p2.x, p2.y);

        double arrowLength = 3;
        double theta = Math.atan2(velocity.getY(), velocity.getX());

        double thetaCWArrow = theta - 5*Math.PI/6;
        Vector2d CWArrow = new Vector2d(
                p2.x + arrowLength * Math.cos(thetaCWArrow),
                p2.y + arrowLength * Math.sin(thetaCWArrow)
        );
        c.strokeLine(p2.x, p2.y, CWArrow.x, CWArrow.y);

        double thetaCCWArrow = theta + 5*Math.PI/6;
        Vector2d CCWArrow = new Vector2d(
                p2.x + arrowLength * Math.cos(thetaCCWArrow),
                p2.y + arrowLength * Math.sin(thetaCCWArrow)
        );
        c.strokeLine(p2.x, p2.y, CCWArrow.x, CCWArrow.y);



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