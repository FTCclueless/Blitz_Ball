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
    double offsetDist;

    double sign;



    double decelThreshold;

    double tempMaxVel;




    public TrapezoidMotionProfile(double maxAccel, double maxVel, double offset) {
        this.maxAccel = maxAccel;
        this.maxVel = maxVel;
        this.offsetDist = offset;




    }

    //when you set target position/distance
    public void setTargetPos(double targetPos, double currentPos, double currentVel) {
        this.targetPos = targetPos;
        distance = targetPos-currentPos;
        startPos = currentPos;
        startVel = currentVel;
        accelTime = (maxVel-currentVel)/maxAccel;
        decelTime = maxVel / maxAccel;
        decelDist = 0.5 * maxAccel * decelTime * decelTime;
        accelDist = 0.5 * maxAccel* accelTime * accelTime + startVel * accelTime; //probably broken maybe

        sign = Math.signum(distance);


        if (accelDist > Math.abs(distance/2)) {
            halfTime = -startVel + Math.sqrt(startVel*startVel + 4*maxAccel*Math.abs(distance/2));
            //halfTime = Math.sqrt(Math.abs(distance/2) / (0.5*maxAccel)) - currentVel/maxAccel;
            tempMaxVel = halfTime*maxAccel + startVel;

            accelTime = halfTime;
            decelTime = tempMaxVel/maxAccel;
            accelDist = 0.5 * maxAccel* accelTime * accelTime + startVel * accelTime; //
            decelDist = maxAccel*decelTime;
        }
        if (Math.abs(decelDist) >= Math.abs(distance) - offsetDist) {
            accelTime = 0;
            accelDist = 0;
        }

        cruiseDist = Math.abs(distance)-decelDist;

        decelThreshold = accelDist + cruiseDist + decelDist;







    }
//every loop to get target velocity
    public double getTargetVel(double currentPos) {
        traveledDist = Math.abs(currentPos-startPos) + offsetDist; //offsetDist is to prevent power from permanently being 0
        TelemetryUtil.packet.put("traveledDist", traveledDist);
        TelemetryUtil.packet.put("accelDist", accelDist);
        TelemetryUtil.packet.put("** cruiseDist", cruiseDist);
        TelemetryUtil.packet.put("** decelThreshold", decelThreshold);
        TelemetryUtil.packet.put(" ** decelDist", decelDist);
        TelemetryUtil.packet.put("***prof dist", distance);
        TelemetryUtil.packet.put("halftime", halfTime);
        TelemetryUtil.packet.put("**accelTime", accelTime);
        TelemetryUtil.packet.put("**tempMaxVel", tempMaxVel);
        TelemetryUtil.packet.put("** startVel", startVel);
        System.out.println(accelDist);


        if (Math.abs(traveledDist) < accelDist + offsetDist || (sign == -1 && traveledDist > accelDist) || (sign == 1 && traveledDist < accelDist))  {
            TelemetryUtil.packet.put("** Vel",maxAccel*(Math.sqrt(2*traveledDist/maxAccel)) * sign + startVel );
            return maxAccel*(Math.sqrt(2*traveledDist/maxAccel)) * sign + startVel;

        }

        else if (Math.abs(traveledDist) < cruiseDist + offsetDist) {
            TelemetryUtil.packet.put("** Vel", maxVel * sign);
            return maxVel * sign;
        }

        else if (Math.abs(traveledDist) < decelThreshold + offsetDist) {
            return maxVel-maxAccel*((2 * traveledDist/maxAccel) * sign);
        }

        else {
            return 0;
        }


    }
}
