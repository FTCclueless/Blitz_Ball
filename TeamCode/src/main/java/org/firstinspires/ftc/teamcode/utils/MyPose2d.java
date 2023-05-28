package org.firstinspires.ftc.teamcode.utils;

public class MyPose2d {
    public double x;
    public double y;
    public double heading;
    public double headingOffset;
    public boolean mustGoToPoint;

    public MyPose2d(double x, double y){
        this(x,y,0,0);
    }

    public MyPose2d(double x, double y, double heading){
        this(x,y,heading,0);
    }

    public MyPose2d(MyPose2d pose2d){
        this.x = pose2d.x;
        this.y = pose2d.y;
        this.heading = pose2d.heading;
        this.headingOffset = pose2d.headingOffset;
    }

    public MyPose2d(double x, double y, double heading, double headingOffset) {
        this.x = x;
        this.y = y;
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
    public double getHeadingOffset(){
        return headingOffset;
    }

    public double getDistanceFromPoint(MyPose2d newPoint) {
        return Math.sqrt(Math.pow((x - newPoint.x),2) + Math.pow((y - newPoint.y),2));
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
