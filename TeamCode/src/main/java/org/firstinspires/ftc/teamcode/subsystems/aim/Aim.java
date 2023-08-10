package org.firstinspires.ftc.teamcode.subsystems.aim;

import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_POSITION;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;

public class Aim {
    public enum State {
        AUTO_AIM,
        MANUAL_AIM
    }

    public State state = State.AUTO_AIM;

    Sensors sensors;

    public Turret turret;
    public Shooter shooter;
    public Hood hood;
    Target target1;
    Target target2;


    Target mainTarget;

    double leftZero = 0; // need to find later
    double rightZero = 0;

    double binHeight = 20; //TODO figure out
    double binRadius = 7.5; //TODO figure out
    public static double shooterHeight = 15; //TODO figure out
    public static double targetHeight = 2; //TODO figure out

    public static double errorRadius = 3;

    public static double shooterComp = 0;

    public Aim(HardwareMap hardwareMap, HardwareQueue hardwareQueue, Sensors sensors) {
        this.turret = new Turret(hardwareMap, hardwareQueue, sensors);
        turret.state = Turret.State.AUTOAIM;
        this.shooter = new Shooter(hardwareMap, hardwareQueue, sensors);
        this.hood = new Hood(hardwareMap);

        this.sensors = sensors;

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
        shooter.setTargetVel(vel + shooterComp);
    }

    public void setHood(double angle) {
        hood.setAngle(angle);
    }

    public void updateTelemetry() {
        Canvas canvas = TelemetryUtil.packet.fieldOverlay();
        if (target1 != null) {
            canvas.setFill("#ff5445"); // Red
            canvas.fillCircle(target1.target.x, target1.target.y, target1.binRadius);
        }
        if (target2 != null) {
            canvas.setFill("#4248fc"); // Blue
            canvas.fillCircle(target2.target.x, target2.target.y, target2.binRadius);
        }
        canvas.setStroke("#f8ff73");
        canvas.strokeLine(
            ROBOT_POSITION.x, ROBOT_POSITION.y,
            ROBOT_POSITION.x + Math.cos(turret.currentAngle + ROBOT_POSITION.heading) * 24,
            ROBOT_POSITION.y + Math.sin(turret.currentAngle + ROBOT_POSITION.heading) * 24
        );

        Pose2d check = shootPose();
        canvas.setFill("#7fff69");
        canvas.fillCircle(check.x, check.y, 3);
    }

    public Pose2d shootPose() {
        double shooterVel = sensors.getShooterVelocity();
        double hoodAngle = hood.getAngle();
        double turretAngle = turret.currentAngle;

        double verticalVel = shooterVel * Math.sin(hoodAngle);
        double forwardVel = shooterVel * Math.cos(hoodAngle);

        double time = (verticalVel + Math.sqrt(Math.pow(verticalVel,2)+4*shooterHeight*386.2205/2))/386.2205; // 9.81 m/s in inches

        double velX = forwardVel * Math.cos(turretAngle + ROBOT_POSITION.heading);
        double velY = forwardVel * Math.sin(turretAngle + ROBOT_POSITION.heading);


        return new Pose2d(
            velX * time + ROBOT_POSITION.x,
            velY * time + ROBOT_POSITION.y
        );
        // return (Math.sqrt(Math.pow(mainTarget.target.x - velX*time,2) + Math.pow(mainTarget.target.y - velY*time,2))) < errorRadius;
    }

    public void setState(State state) {
        this.state = state;
    }



    public void update() {
        updateTelemetry();

        switch (state) {
            case AUTO_AIM:
                mainTarget.update();
                shooter.setTargetVel(mainTarget.targetShooterVel + shooterComp);
                turret.setTargetAngle(mainTarget.targetTurretAngle - ROBOT_POSITION.heading);//,mainTarget.futureTurretOffset);
                //leftShooterHood.setAngle(mainTarget.targetShooterAngle);
                //rightShooterHood.setAngle(mainTarget.targetShooterAngle);
                turret.update();
                shooter.update();
                break;

            case MANUAL_AIM:
                turret.update();
                shooter.update();
                break;
        }
    }

}
