package org.firstinspires.ftc.teamcode.subsystems.drive.aim;

import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_POSITION;
import static org.firstinspires.ftc.teamcode.utils.Globals.ROBOT_VELOCITY;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.MotorPriority;
import org.firstinspires.ftc.teamcode.utils.MyServo;
import org.firstinspires.ftc.teamcode.utils.Pose2d;

import java.util.ArrayList;

public class AutoAim {

    Turret turret;
    Shooter shooter;
    Target target1;
    Target target2;

    MyServo leftShooterHood;
    MyServo rightShooterHood;

    Target mainTarget;

    double leftZero = 0; // need to find later
    double rightZero = 0;

    double binHeight = 10; // TODO find this
    double binRadius = 2; //TODO find this
    public static double shooterHeight = 15;
    public static double targetHeight = 2;



    public AutoAim(HardwareMap hardwareMap, ArrayList<MotorPriority> motorPriorities, Sensors sensors) {
        this.turret = new Turret(hardwareMap, motorPriorities, sensors);
        turret.turretState = TurretState.AUTOAIM;
        this.shooter = new Shooter(hardwareMap, motorPriorities, sensors);
        //leftShooterHood = new MyServo(hardwareMap.get(Servo.class, "leftShooterHood"),"axon", leftZero, false);
        //rightShooterHood = new MyServo(hardwareMap.get(Servo.class, "rightShooterHood"), "axon", rightZero, true);

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



    public void update() {
        mainTarget.update();
        shooter.setTargetVel(mainTarget.targetShooterVel);
        Log.e("targetAngle in autoaim", mainTarget.targetTurretAngle-ROBOT_POSITION.heading + "" );
        turret.setTargetAngle(mainTarget.targetTurretAngle - ROBOT_POSITION.heading);//,mainTarget.futureTurretOffset);
        //leftShooterHood.setAngle(mainTarget.targetShooterAngle);
        //rightShooterHood.setAngle(mainTarget.targetShooterAngle);
        turret.update();
        shooter.update();

    }

}
