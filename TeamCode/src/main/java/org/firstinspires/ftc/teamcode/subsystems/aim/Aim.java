package org.firstinspires.ftc.teamcode.subsystems.aim;

import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_POSITION;

import android.util.Log;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.subsystems.Ball;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;

import java.util.ArrayList;

public class Aim {
    public enum State {
        AUTO_AIM,
        MANUAL_AIM,
        OFF
    }

    public State state = State.AUTO_AIM;

    Sensors sensors;

    public Turret turret;
    public Shooter shooter;
    public Hood hood;
    public Transfer transfer;
    private ArrayList<Target> targets = new ArrayList<>();


    Target mainTarget;

    double leftZero = 0; // need to find later
    double rightZero = 0;

    double binHeight = 16; //TODO figure out
    double binRadius = 7.5; //TODO figure out
    public static double shooterHeight = 15; //TODO figure out
    public static double targetHeight = 2; //TODO figure out

    public static double errorRadius = 3;

    public static double shooterComp = 0;

    public boolean shoot = false;

    public Aim(HardwareMap hardwareMap, HardwareQueue hardwareQueue, Sensors sensors) {
        this.turret = new Turret(hardwareMap, hardwareQueue, sensors);
        turret.state = Turret.State.AUTOAIM;
        this.shooter = new Shooter(hardwareMap, hardwareQueue, sensors);
        transfer = new Transfer(hardwareMap, hardwareQueue, sensors);
        this.hood = new Hood(hardwareMap, hardwareQueue);

        this.sensors = sensors;

    }

    public void setTarget(int index, double x, double y, Ball color) {
        this.setTarget(index, new Pose2d(x, y, 0), color);
    }

    public void setTarget(int index, Pose2d target, Ball color) {
        targets.set(index, new Target(target, binHeight, binRadius, shooterHeight, targetHeight, color));
    }

    public void addTarget(double x, double y, Ball color) {
        targets.add(new Target(new Pose2d(x,y,0), binHeight, binRadius, shooterHeight, targetHeight, color));
    }

    public ArrayList<Target> getAvailableTargets() {
        ArrayList<Target> ret = new ArrayList<>();
        for (Target target : targets) {
            double t = Math.atan2(target.target.y, target.target.x);
            if (ROBOT_POSITION.heading + Turret.maxRotation > t && ROBOT_POSITION.heading - Turret.maxRotation < t) {
                ret.add(target);
            }
        }
        return ret;
    }

    public void setMainTarget(int index) {
        mainTarget = targets.get(index);
    }

    public void setMainTarget(Target t) {
        mainTarget = t;
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
        for (Target t : targets) {
            canvas.setFill("#ff5445");
            canvas.fillCircle(t.target.x, t.target.y, t.binRadius);
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
        double shooterVel = shooter.getTargetVel();
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
        switch (state) {
            case AUTO_AIM:
                turret.state = Turret.State.AUTOAIM;
                shooter.state = Shooter.State.AUTO_AIM;
                transfer.state = Transfer.State.READ_BEAMBREAK;
                break;
            case MANUAL_AIM:
                turret.state = Turret.State.AUTOAIM;
                shooter.state = Shooter.State.AUTO_AIM;
                transfer.state = Transfer.State.HOLD;
                break;
            case OFF:
                turret.state = Turret.State.OFF;
                shooter.state = Shooter.State.OFF;
                transfer.state = Transfer.State.HOLD;
                break;
        }
    }



    public void update() {
        updateTelemetry();

        switch (state) {
            case AUTO_AIM:
                mainTarget.update();
                shooter.setTargetVel(mainTarget.targetShooterVel + shooterComp);
                TelemetryUtil.packet.put("aimVel", mainTarget.targetShooterVel);
                turret.setTargetAngle(mainTarget.targetTurretAngle - ROBOT_POSITION.heading);//,mainTarget.futureTurretOffset);
                hood.setAngle(mainTarget.targetShooterAngle);
                TelemetryUtil.packet.put("hoodAngle", mainTarget.targetShooterAngle);
                //leftShooterHood.setAngle(mainTarget.targetShooterAngle);
                //rightShooterHood.setAngle(mainTarget.targetShooterAngle);

                if (transfer.currentBall != Ball.EMPTY) {
                    for (Target t : getAvailableTargets()) {
                        if (t.color == transfer.currentBall) {
                            setMainTarget(t);
                            shoot = true;
                            transfer.state = Transfer.State.HOLD;
                        }
                    }
                    if (!shoot) {
                        transfer.state = Transfer.State.EJECT;
                    }
                }

                if (shoot) {
                    if (Math.sqrt(Math.pow(mainTarget.target.x - shootPose().x,2) + Math.pow(mainTarget.target.y - shootPose().y, 2)) < errorRadius) {
                        transfer.state = Transfer.State.SHOOT;
                        shoot = false;
                    }
                }



                turret.update();
                shooter.update();
                transfer.update();
                for (Target t : targets) {
                    t.update();
                }
                break;

            case MANUAL_AIM:

            case OFF:
                turret.update();
                shooter.update();
                transfer.update();
                break;
        }
    }


    boolean pastRight = false;
    boolean pastLeft = false;
    public void manual(Gamepad gamepad2) {
        turret.setPower(0.4 * gamepad2.left_stick_x);
        hood.setAngle(0.05 * gamepad2.left_stick_y + hood.getAngle());
        //shooter.setTargetVel(shooter.getSpeed() + gamepad2.right_stick_y * 0.5);

        if (gamepad2.right_trigger > 0.3 && !pastRight) {
            pastRight = true;
            transfer.shootBall();
        }
        else {
            pastRight = false;
            if (gamepad2.left_trigger > 0.3 && !pastLeft) {
                transfer.ejectBall();
                pastLeft = true;
            }
            else {
                pastLeft = false;
            }
        }



        turret.update();
        shooter.update();
        transfer.update();
    }

    public void manualAutoAim(Gamepad gamepad2) {
        mainTarget.update();
        shooter.setTargetVel(mainTarget.targetShooterVel + shooterComp);
        TelemetryUtil.packet.put("aimVel", mainTarget.targetShooterVel);
        turret.setTargetAngle(mainTarget.targetTurretAngle - ROBOT_POSITION.heading);

        if (gamepad2.right_trigger > 0.3 && !pastRight) {
            pastRight = true;
            transfer.shootBall();
        }
        else {
            pastRight = false;
            if (gamepad2.left_trigger > 0.3 && !pastLeft) {
                transfer.ejectBall();
                pastLeft = true;
            }
            else {
                pastLeft = false;
            }
        }

        turret.update();
        shooter.update();
        transfer.update();
    }

}
