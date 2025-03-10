package org.firstinspires.ftc.teamcode.opmodes.calibration;

///////////////////////////////////
// CREDIT: ROADRUNNER QUICKSTART //
///////////////////////////////////

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public final class Drawing {
    private Drawing() {}

    public static double HEATMAP_COLOR_THRESHOLD = 1;

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

    public static void drawSample(Canvas c, com.arcrobotics.ftclib.geometry.Pose2d t, String color) {
        Pose2d t_rr = new Pose2d(t.getX(), t.getY(), t.getHeading());

        c.setStrokeWidth(1);
        c.setStroke(color);
        c.strokeLine(
                t_rr.position.x+1.5, t_rr.position.y+2.5,
                t_rr.position.x-1.5, t_rr.position.y+2.5
        );
        c.strokeLine(
                t_rr.position.x-1.5, t_rr.position.y+2.5,
                t_rr.position.x-1.5, t_rr.position.y-2.5
        );
        c.strokeLine(
                t_rr.position.x-1.5, t_rr.position.y-2.5,
                t_rr.position.x+1.5, t_rr.position.y-2.5
        );
        c.strokeLine(
                t_rr.position.x+1.5, t_rr.position.y-2.5,
                t_rr.position.x+1.5, t_rr.position.y+2.5
        );

    }

    public static void drawSampleHeatmap(Canvas canvas, double[][] sample_probability_distribution, double FIELD_RESOLUTION, int RESOLUTION_N, Telemetry telemetry) {
        canvas.setStrokeWidth(1);

        float h0 = 0.784167f;
        float s0 = 0.722f;
        float v0 = 0.212f;

        float h1 = 0.14972f;
        float s1 = 0.854f;
        float v1 = 0.992f;

        float dh = h1-h0;
        float ds = s1-s0;
        float dv = v1-v0;

        for (int r = 4*RESOLUTION_N/9; r < 5*RESOLUTION_N/9; r++) {
            for (int c = RESOLUTION_N/2; c < 6*RESOLUTION_N/6; c++) {
                if (sample_probability_distribution[r][c] > 0.05) {
                    float t = (float) Math.min(sample_probability_distribution[r][c] / HEATMAP_COLOR_THRESHOLD, 1);
                    String color = "#" + hsvToRgb(
                            h0 + t * dh,
                            s0 + t * ds,
                            v0 + t * dv
                    );
                    canvas.setFill(color);
                    double px = FIELD_RESOLUTION * (c - (double) RESOLUTION_N / 2);
                    double py = FIELD_RESOLUTION * (r - (double) RESOLUTION_N / 2);
                    canvas.fillRect(px, py, FIELD_RESOLUTION, FIELD_RESOLUTION);
                }
            }
        }


    }



    public static String hsvToRgb(float hue, float saturation, float value) {

        int h = (int)(hue * 6);
        float f = hue * 6 - h;
        float p = value * (1 - saturation);
        float q = value * (1 - f * saturation);
        float t = value * (1 - (1 - f) * saturation);

        switch (h) {
            case 0: return rgbToString(value, t, p);
            case 1: return rgbToString(q, value, p);
            case 2: return rgbToString(p, value, t);
            case 3: return rgbToString(p, q, value);
            case 4: return rgbToString(t, p, value);
            case 5: return rgbToString(value, p, q);
            default: throw new RuntimeException("Something went wrong when converting from HSV to RGB. Input was " + hue + ", " + saturation + ", " + value);
        }
    }

    public static String rgbToString(float r, float g, float b) {
        String rs = Integer.toHexString((int)(r * 256));
        if (rs.length()==1) rs = "0"+rs;
        String gs = Integer.toHexString((int)(g * 256));
        if (gs.length()==1) gs = "0"+gs;
        String bs = Integer.toHexString((int)(b * 256));
        if (bs.length()==1) bs = "0"+bs;
        return rs + gs + bs;
    }
}