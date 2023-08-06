package org.firstinspires.ftc.teamcode.subsystems.aim;

import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_POSITION;
import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_VELOCITY;

import android.util.Log;

import org.firstinspires.ftc.teamcode.utils.AngleUtil;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.Vector2;

public class Target {
    Pose2d target;
    double binHeight;
    double binRadius;
    double targetHeight;
    double shooterHeight;


    public double targetShooterAngle = 0;
    public double targetTurretAngle = 0;
    public double targetShooterVel = 0;


    public int counter = 0;
    private double[] pastHeadings = new double[] {0,0,0,0,0};
    private double[] pastLoopTimes = new double[] {0,0,0,0,0};
    public double pastHeadingSum = 0;
    public double futureTurretOffset = 0;

    double time = 0;
    double lastTime = 0;


    public Target(Pose2d target, double binHeight, double binRadius, double shooterHeight, double targetHeight) {
        this.target = target;
        this.binHeight = binHeight;
        this.binRadius = binRadius;
        this.shooterHeight = shooterHeight;
        this.targetHeight = targetHeight;

        for (int i = 0; i < 5; i ++){
            pastLoopTimes[i] = (double)System.nanoTime();
        }
    }


    public void update() {
        //y2 = bin height
        //y3 = target Height
        //c = shooter height
        double turretOffsetX = -4.0;
        double turretOffsetY = 0;
        Pose2d turretPos = new Pose2d(
                ROBOT_POSITION.x + Math.cos(ROBOT_POSITION.heading) * turretOffsetX,
                ROBOT_POSITION.y + Math.sin(ROBOT_POSITION.heading) * turretOffsetX,
                ROBOT_POSITION.heading
        );
        time = System.nanoTime()/1000000000.0;
        double binDistance = Math.sqrt(Math.pow(target.x-turretPos.x,2) + Math.pow(target.y-turretPos.y,2));
        double math = Math.sqrt((binDistance-binRadius)/((binHeight-targetHeight/binRadius - (targetHeight - shooterHeight)/binDistance)));
        double targetVelMag = Math.sqrt(9.81/2)*math;
        double targetVelZ = Math.sqrt(9.81/2)*((targetHeight-shooterHeight)/binDistance * math + binDistance/math);

        Vector2 launchVector = new Vector2(target.x-turretPos.x, target.y-turretPos.y);
        launchVector.norm();
        launchVector.mul(targetVelMag);
        launchVector.add(new Vector2(-ROBOT_VELOCITY.x, -ROBOT_VELOCITY.y));


        Log.e("launchx", launchVector.x + "");
        Log.e("launchy", launchVector.y + "");
        targetTurretAngle = Math.atan2(launchVector.y, launchVector.x);
        Log.e("target", targetTurretAngle + "");

        targetShooterAngle = Math.atan2(targetVelZ, launchVector.mag());
        targetShooterVel = Math.sqrt(Math.pow(launchVector.mag(),2) + Math.pow(targetVelZ,2));

        pastHeadings[counter%5] = targetTurretAngle;
        double deltaHeading = targetTurretAngle - pastHeadings[(counter+4)%5];
        deltaHeading = AngleUtil.clipAngle(deltaHeading);

        pastLoopTimes[counter%5] = (double)System.nanoTime();
        double deltaTime = pastLoopTimes[counter%5] - pastLoopTimes[(counter+4)%5];
        deltaTime /= 1.0e9;

        futureTurretOffset = deltaHeading/deltaTime + ROBOT_VELOCITY.heading;
        futureTurretOffset = 0;
        lastTime = time;
        counter ++;
    }
}
