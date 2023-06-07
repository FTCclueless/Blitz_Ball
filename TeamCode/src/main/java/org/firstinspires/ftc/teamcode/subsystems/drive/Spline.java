package org.firstinspires.ftc.teamcode.subsystems.drive;

import android.util.Log;

import org.firstinspires.ftc.teamcode.utils.MyPose2d;

import java.util.ArrayList;

public class Spline {
    public ArrayList<MyPose2d> points = new ArrayList<>();
    public double headingOffset = 0;
    double inchesPerNewPointGenerated = 2.0;

    public Spline(MyPose2d startPose) {
        points.add(startPose);
    }

    public Spline addPoint (MyPose2d newPoint) {
        // https://www.desmos.com/calculator/yi3jovk0hp

        double[] xCoefficents = new double[4];
        double[] yCoefficents = new double[4];

        MyPose2d lastPoint = points.get(points.size()-1); // when you add a new spline the last point becomes the starting point for the new spline

        double arbitraryVelocity = 1.25*Math.sqrt(Math.pow((lastPoint.x - newPoint.x),2) + Math.pow((lastPoint.y - newPoint.y),2));

        xCoefficents[0] = lastPoint.x;
        xCoefficents[1] = arbitraryVelocity * Math.cos(lastPoint.heading);
        xCoefficents[2] = 3*newPoint.x - arbitraryVelocity*Math.cos(newPoint.heading) - 2*xCoefficents[1] - 3*xCoefficents[0];
        xCoefficents[3] = newPoint.x - xCoefficents[0] - xCoefficents[1] - xCoefficents[2];

        yCoefficents[0] = lastPoint.y;
        yCoefficents[1] = arbitraryVelocity * Math.sin(lastPoint.heading);
        yCoefficents[2] = 3*newPoint.y - arbitraryVelocity*Math.sin(newPoint.heading) - 2*yCoefficents[1] - 3*yCoefficents[0];
        yCoefficents[3] = newPoint.y - yCoefficents[0] - yCoefficents[1] - yCoefficents[2];

        for (double time = 0.0; time < 1.0; time+=0.001) {
            MyPose2d point = new MyPose2d(0,0,0);

            point.x = xCoefficents[0] + xCoefficents[1]*time + xCoefficents[2]*time*time + xCoefficents[3]*time*time*time;
            point.y = yCoefficents[0] + yCoefficents[1]*time + yCoefficents[2]*time*time + yCoefficents[3]*time*time*time;

            if(lastPoint.getDistanceFromPoint(point) > inchesPerNewPointGenerated) { // new point every two inches
                // gets the velocity because the derivative of position = velocity
                double velX = xCoefficents[1] + 2.0*xCoefficents[2]*time + 3.0*xCoefficents[3]*time*time;
                double velY = yCoefficents[1] + 2.0*yCoefficents[2]*time + 3.0*yCoefficents[3]*time*time;

                // gets the acceleration which is second derivative of position
                double accelX = 2.0 + 6.0*xCoefficents[3]*time;
                double accelY = 2.0 + 6.0*xCoefficents[3]*time;

                double radius = calculateInstantRadius(velX, velY, accelX, accelY)/1.5;
                point.setRadius(radius);

                // heading is equal to the inverse tangent of velX and velY because velX and velY have a magnitude and a direction and soh cah toa
                point.heading = Math.atan2(velY,velX);
                point.headingOffset = headingOffset;
                point.clipAngle();

                points.add(point);
                lastPoint = point;
            }
        }
        newPoint.headingOffset = headingOffset;
        points.add(newPoint);

        return this;
    }

    double minimumRobotThresholdFromEndPointInX = 1;
    double minimumRobotThresholdFromEndPointInY = 4;
    double minimumRobotTurningThresholdFromEndPoint = Math.toRadians(5);

    // r = (dx^2 + dy^2)^1.5/(ddy*dx-ddx*dy)
    public double calculateInstantRadius (double velX, double velY, double accelX, double accelY) {
        return Math.abs(Math.pow(Math.pow(velX,2) + Math.pow(velY,2),1.5)/(accelY*velX-velY*accelX));
    }

    public MyPose2d getErrorFromNextPoint(MyPose2d currentRobotPose) {
        if(points.size() == 0){
            return null;
        }

        // loops through all of the points and removes any points that are within the robotsMinimumDistanceFromPoint. When we remove a point the nextPoint becomes the 0th index so we don't increment anything
        while(points.size() > 1 && !points.get(0).mustGoToPoint && points.get(0).getDistanceFromPoint(currentRobotPose) < points.get(0).getRadius()) {
            points.remove(0);
        }

        // global error to relative error (https://drive.google.com/file/d/1bqHU0ZHKN2yaxgf4M6FBV36Sv0TX0C1g/view?usp=sharing)
        MyPose2d globalError = new MyPose2d(points.get(0).x - currentRobotPose.x, points.get(0).y - currentRobotPose.y);
        MyPose2d relativeError = new MyPose2d(
                globalError.x*Math.cos(currentRobotPose.heading) + globalError.y*Math.sin(currentRobotPose.heading),
                globalError.y*Math.cos(currentRobotPose.heading) - globalError.x*Math.sin(currentRobotPose.heading),
                points.get(0).heading-currentRobotPose.heading);
        relativeError.clipAngle();

        // checking if we have finished the spline

        if ((Math.abs(relativeError.x) < minimumRobotThresholdFromEndPointInX)
                && (Math.abs(relativeError.y) < minimumRobotThresholdFromEndPointInY)
                && (Math.abs(relativeError.heading) < minimumRobotTurningThresholdFromEndPoint)) {
            points.remove(0);
            if (points.size() == 0) {
                return null;
            }
        }

        return relativeError;
    }

    public MyPose2d end() {
        return points.get(points.size()-1);
    }

    // When building splines you must put this function before the section you want to be reversed
    public Spline setReversed(boolean setReversed) {
        headingOffset = (setReversed) ? Math.toRadians(180) : 0;
        MyPose2d reversedLastPoint = getLastPoint();
        reversedLastPoint.heading += headingOffset;
        reversedLastPoint.headingOffset = headingOffset;
        reversedLastPoint.mustGoToPoint = false;
        reversedLastPoint.clipAngle();
        points.add(reversedLastPoint);
        return this;
    }

    public MyPose2d getLastPoint () {
        if (points.size() > 0) {
            return points.get(points.size()-1);
        }
        return new MyPose2d(0,0,0);
    }

    public Spline turn (double angle) {
        MyPose2d lastPoint = getLastPoint();
        lastPoint.heading = angle;
        points.add(lastPoint);
        return this;
    }

    public Spline mustGoToPoint () {
        MyPose2d lastPoint = getLastPoint();
        lastPoint.mustGoToPoint = true;
        points.add(lastPoint);
        return this;
    }
}
