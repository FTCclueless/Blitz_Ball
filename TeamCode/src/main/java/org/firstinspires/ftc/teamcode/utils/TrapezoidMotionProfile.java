package org.firstinspires.ftc.teamcode.utils;

public class TrapezoidMotionProfile {
    double maxAccel;
    double maxVel;
    double startPos;
    double targetPos;

    double accelTime = 0;
    double accelDist;
    double cruiseDist;
    double cruiseTime;
    double decelDist;
    double decelTime;
    double traveledDist;
    double distance;

    double halfTime;

    double startVel;

    double sign;







    public TrapezoidMotionProfile(double maxAccel, double maxVel) {
        this.maxAccel = maxAccel;
        this.maxVel = maxVel;




    }

    public void setDistance(double targetPos, double currentPos, double currentVel) {
        this.targetPos = targetPos;
        distance = targetPos-currentPos;
        startPos = currentPos;
        startVel = currentVel;
        accelTime = (maxVel-currentVel)/maxAccel;
        decelTime = maxVel / maxAccel;
        decelDist = 0.5 * maxAccel * decelTime * decelTime;
        accelDist = 0.5 * maxAccel* accelTime * accelTime + startVel * accelTime; //probably broken maybe

        sign = Math.signum(distance);

        double tempMaxVel;
        if (accelDist > Math.abs(distance/2)) {
            halfTime = Math.sqrt(Math.abs(distance/2) / (0.5*maxAccel)) - currentVel/maxAccel;
            tempMaxVel = halfTime*maxAccel;

            accelTime = tempMaxVel/maxAccel;
            accelDist = 0.5 * maxAccel* accelTime * accelTime + startVel * accelTime; //
        }

        cruiseDist = Math.abs(distance)-accelDist;

        decelDist = accelDist + cruiseDist + decelTime;







    }

    public double getTargetVel(double currentPos) {
        traveledDist = currentPos-startPos;

        if (Math.abs(traveledDist) < accelDist || (sign == -1 && traveledDist > accelDist) || (sign == 1 && traveledDist < accelDist))  {
            return maxAccel*(Math.sqrt(2*traveledDist/maxAccel)) * sign;
        }

        else if (Math.abs(traveledDist) < cruiseTime) {
            return maxVel * sign;
        }

        else if (Math.abs(traveledDist) < decelDist) {
            return maxVel-maxAccel*((2 * traveledDist/maxAccel) * sign);
        }

        else {
            return 0;
        }


    }
}
