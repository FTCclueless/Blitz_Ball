package org.firstinspires.ftc.teamcode.subsystems.aim;

import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_POSITION;

import android.util.Log;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.MotorPriority;
import org.firstinspires.ftc.teamcode.utils.Pose2d;

import java.util.ArrayList;

public class Aim {
    public AimState aimState = AimState.AUTO_AIM;

    public Turret turret;
    public Shooter shooter;
    public Hood hood;
    Target target1;
    Target target2;


    Target mainTarget;

    double leftZero = 0; // need to find later
    double rightZero = 0;

    double binHeight = 16;
    double binRadius = 7.5;
    public static double shooterHeight = 15;
    public static double targetHeight = 2;



    public Aim(HardwareMap hardwareMap, ArrayList<MotorPriority> motorPriorities, Sensors sensors) {
        this.turret = new Turret(hardwareMap, motorPriorities, sensors);
        turret.turretState = TurretState.AUTOAIM;
        this.shooter = new Shooter(hardwareMap, motorPriorities, sensors);
        this.hood = new Hood(hardwareMap);

    }

    public void setTarget1(double x, double y) {
        target1 = new Target(new Pose2d(x,y,0), binHeight, binRadius, shooterHeight, targetHeight);
    }

    public void setTarget1(Pose2d target) {
        target1 = new Target(target, binHeight, binRadius, shooterHeight, targetHeight);
    }

    public void setTarget2(double x, double y) {
        target2 = new Target(new Pose2d(x,y,0),binHeight,binRadius,shooterHeight,targetHeight);
    }

    public void setTarget2(Pose2d target) {
        target2 = new Target(target, binHeight,binRadius,shooterHeight,targetHeight);
    }

    public void switchTarget() {
        if (mainTarget == target1) {
            mainTarget = target2;
        }
        else {
            mainTarget = target1;
        }
    }

    public void setMainTarget(int target) {
        if (target == 1) {
            mainTarget = target1;
        }
        else if (target == 2) {
            mainTarget = target2;
        }
        else {
            System.out.println("bad bin target");
        }
    }

    public void setTurret(double angle) {
        turret.setTargetAngle(angle);
    }

    public void setShooter(double vel) {
        shooter.setTargetVel(vel);
    }

    public void setHood(double angle) {
        hood.setAngle(angle);
    }



    public void update() {
        switch (aimState) {
            case AUTO_AIM:
                mainTarget.update();
                shooter.setTargetVel(mainTarget.targetShooterVel);
                turret.setTargetAngle(mainTarget.targetTurretAngle - ROBOT_POSITION.heading);//,mainTarget.futureTurretOffset);
                //leftShooterHood.setAngle(mainTarget.targetShooterAngle);
                //rightShooterHood.setAngle(mainTarget.targetShooterAngle);
                turret.update();
                shooter.update();

            case MANUAL_AIM:
                turret.update();
                shooter.update();
        }
    }

}
