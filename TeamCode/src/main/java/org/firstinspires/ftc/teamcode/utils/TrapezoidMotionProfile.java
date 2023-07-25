package org.firstinspires.ftc.teamcode.utils;

public class TrapezoidMotionProfile {
    double maxAccel;
    double maxVel;
    double distance;
    double elapsedTime;



    public TrapezoidMotionProfile(double maxAccel, double maxVel, double distance, double elapsedTime) {
        this.maxAccel = maxAccel;
        this.maxVel = maxVel;
        this.distance = distance;
        this.elapsedTime = elapsedTime;


    }
}
