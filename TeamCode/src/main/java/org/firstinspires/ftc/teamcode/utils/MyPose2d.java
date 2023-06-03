package org.firstinspires.ftc.teamcode.utils;

public class MyPose2d {
    public double x;
    public double y;
    public double radius;
    public double heading;
    public double headingOffset;
    public boolean mustGoToPoint;

    public MyPose2d(double x, double y){
        this(x,y,0,0,0);
    }

    public MyPose2d(double x, double y, double heading){
        this(x,y,heading,0,0);
    }

    public MyPose2d(double x, double y, double heading, double headingOffset, double radius) {
        this.x = x;
        this.y = y;
        this.radius = radius;
        this.heading = heading;
        this.headingOffset = headingOffset;
        this.mustGoToPoint = false;
    }

    public double getX(){ return x; }
    public double getY(){
        return y;
    }
    public double getHeading(){
        return heading;
    }

    public static final double minDistanceFromPoint = 4.0;
    public static final double maxDistanceFromPoint = 14.0;

    public void setRadius(double radius) {
        this.radius = Math.min(maxDistanceFromPoint, Math.max(minDistanceFromPoint, radius));;
    }

    public double getRadius() {
        return Math.min(maxDistanceFromPoint, Math.max(minDistanceFromPoint, radius));
    }

    public double getDistanceFromPoint(MyPose2d newPoint) { // distance equation
        return Math.sqrt(Math.pow((x - newPoint.x),2) + Math.pow((y - newPoint.y),2));
    }

    public double getErrorInX(MyPose2d newPoint) { // distance equation
        return Math.abs(x - newPoint.x);
    }

    public double getErrorInY(MyPose2d newPoint) { // distance equation
        return Math.abs(y - newPoint.y);
    }

    public double getAngleDifference(MyPose2d newPoint) {
        newPoint.heading = this.heading - newPoint.heading + headingOffset;
        newPoint.clipAngle();
        return Math.abs(newPoint.heading);
    }

    public void clipAngle() {
        while (Math.abs(this.heading) > Math.PI) {
            this.heading -= Math.PI * 2.0 * Math.signum(this.heading);
        }
    }
}
