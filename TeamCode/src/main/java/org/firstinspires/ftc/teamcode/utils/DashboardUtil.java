package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.canvas.Canvas;

import org.firstinspires.ftc.teamcode.subsystems.drive.Spline;

import java.util.List;

/**
 * Set of helper functions for drawing Road Runner paths and trajectories on dashboard canvases.
 */
public class DashboardUtil {
    private static final double ROBOT_RADIUS = 9; // in
    private static final double POLE_RADIUS = 0.5; // in

    public static void drawPoseHistory(Canvas canvas, List<Pose> poseHistory) {
        double[] xPoints = new double[poseHistory.size()];
        double[] yPoints = new double[poseHistory.size()];
        for (int i = 0; i < poseHistory.size(); i++) {
            Pose pose = poseHistory.get(i);
            xPoints[i] = pose.getX();
            yPoints[i] = pose.getY();
        }
        canvas.strokePolyline(xPoints, yPoints);
    }

    public static void drawSampledPath(Canvas canvas, Spline spline) {
        int samples = spline.poses.size();
        double[] xPoints = new double[samples];
        double[] yPoints = new double[samples];
        for (int i = 0; i < samples; i++) {
            Pose pose = spline.poses.get(i);
            xPoints[i] = pose.getX();
            yPoints[i] = pose.getY();
        }
        canvas.strokePolyline(xPoints, yPoints);
        canvas.strokeCircle(spline.getLastPoint().x, spline.getLastPoint().y, 6);
    }

    public static void drawRobot(Canvas canvas, Pose pose) {
        canvas.strokeCircle(pose.getX(), pose.getY(), ROBOT_RADIUS);
        canvas.strokeCircle(pose.getX(), pose.getY(), 0.5);
        Pose v = new Pose(Math.cos(pose.heading)*ROBOT_RADIUS, Math.sin(pose.heading)*ROBOT_RADIUS);
        double x1 = pose.getX() + v.getX() / 2, y1 = pose.getY() + v.getY() / 2;
        double x2 = pose.getX() + v.getX(), y2 = pose.getY() + v.getY();
        canvas.strokeLine(x1, y1, x2, y2);
    }

    public static void drawPole(Canvas canvas, Pose pose) {
        canvas.strokeCircle(pose.getX(), pose.getY(), POLE_RADIUS);
    }
}
