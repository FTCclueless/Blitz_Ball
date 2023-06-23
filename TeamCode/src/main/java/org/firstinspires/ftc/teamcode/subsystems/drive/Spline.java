package org.firstinspires.ftc.teamcode.subsystems.drive;

import org.firstinspires.ftc.teamcode.utils.Pose2d;

import java.util.ArrayList;

public class Spline {
    public ArrayList<Pose2d> poses = new ArrayList<>();

    public final double inchesPerNewPointGenerated;

    public Spline(org.firstinspires.ftc.teamcode.utils.Pose2d p, double inchesPerNewPointGenerated) {
        poses.add(p);

        this.inchesPerNewPointGenerated = inchesPerNewPointGenerated;
    }

    public Spline addSpline(org.firstinspires.ftc.teamcode.utils.Pose2d p) {
        // https://www.desmos.com/calculator/yi3jovk0hp

        double[] xCoefficents = new double[4];
        double[] yCoefficents = new double[4];

        Pose2d lastPoint = poses.get(poses.size()-1); // when you add a new spline the last point becomes the starting point for the new spline

        double arbitraryVelocity = 1.25*Math.sqrt(Math.pow((lastPoint.x - p.x),2) + Math.pow((lastPoint.y - p.y),2));



        double velX = 0, velY = 0, accelX = 0, accelY = 0   ;

        for (double time = 0.0; time < 1.0; time+=0.001) {
            Pose2d point = new Pose2d(0,0,0);


            xCoefficents[0] = lastPoint.x;
            xCoefficents[1] = arbitraryVelocity * Math.cos(lastPoint.heading);
            xCoefficents[2] = 3*p.x - arbitraryVelocity*Math.cos(p.heading) - 2*xCoefficents[1] - 3*xCoefficents[0];
            xCoefficents[3] = p.x - xCoefficents[0] - xCoefficents[1] - xCoefficents[2];

            yCoefficents[0] = lastPoint.y;
            yCoefficents[1] = arbitraryVelocity * Math.sin(lastPoint.heading);
            yCoefficents[2] = 3*p.y - arbitraryVelocity*Math.sin(p.heading) - 2*yCoefficents[1] - 3*yCoefficents[0];
            yCoefficents[3] = p.y - yCoefficents[0] - yCoefficents[1] - yCoefficents[2];


            if(lastPoint.getDistanceFromPoint(point) > inchesPerNewPointGenerated) { // new point every two inches
                point.x = xCoefficents[0] + xCoefficents[1]*time + xCoefficents[2]*time*time + xCoefficents[3]*time*time*time;
                point.y = yCoefficents[0] + yCoefficents[1]*time + yCoefficents[2]*time*time + yCoefficents[3]*time*time*time;

                // gets the velocity because the derivative of position = velocity
                velX = xCoefficents[1] + 2.0*xCoefficents[2]*time + 3.0*xCoefficents[3]*time*time;
                velY = yCoefficents[1] + 2.0*yCoefficents[2]*time + 3.0*yCoefficents[3]*time*time;

                // gets the acceleration which is second derivative of position
                accelX = 2.0 + 6.0*xCoefficents[3]*time;
                accelY = 2.0 + 6.0*xCoefficents[3]*time;

                // heading is equal to the inverse tangent of velX and velY because velX and velY have a magnitude and a direction and soh cah toa
                point.heading = Math.atan2(velY,velX);
                point.clipAngle();

                poses.add(point);

                lastPoint = point;
            }
        }
        poses.add(p);

        return this;
    }

    public org.firstinspires.ftc.teamcode.utils.Pose2d getLastPoint() {
        if (poses.size() > 0) {
            return poses.get(poses.size() - 1);
        }
        return null;
    }
}