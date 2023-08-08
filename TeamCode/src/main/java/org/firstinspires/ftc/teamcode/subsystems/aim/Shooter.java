package org.firstinspires.ftc.teamcode.subsystems.aim;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.MotorPriority;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

import java.util.ArrayList;

@Config
public class Shooter {
    DcMotorEx shooter;
    Sensors sensors;

    ArrayList<MotorPriority> motorPriorities;



    private final double gearRatio = 46.0/9.0;
    private final double radius = 3.25;
    private final double ticksPerRadian = 145.090909091 / gearRatio / (2.0*Math.PI);
    private final double powPerVel = 0.00050180086; //0.002355662527748475; // TODO
    private final double kStatic = 0.12353905285769784; //0.2194966041039161; // TODO
    public static double kAccel = 4;
    public double maxVelocity = (1.0 - kStatic) / powPerVel;
    public double shooterMaxPower = 0;
    private double speed = 0;

    public double targetVelocity;
    public double shooterCurrentPower;
    public double shooterErrorPower;

    public Shooter(HardwareMap hardwareMap, ArrayList<MotorPriority> motorPriorities, Sensors sensors) {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setDirection(DcMotorEx.Direction.REVERSE);

        speed = sensors.getShooterVelocity() / ticksPerRadian * radius;
        this.sensors = sensors;
        this.motorPriorities = motorPriorities;
        motorPriorities.add(new MotorPriority(shooter, 3, 5));
        shooterCurrentPower = 0;
    }

    public void setTargetVel(double velocity) {
        targetVelocity = velocity;
    }

    public double getTargetVel() {
        return targetVelocity;
    }

    public double getSpeed() {
        return speed;
    }

    private double feedForward() {
        return targetVelocity * powPerVel + (targetVelocity - speed) / maxVelocity * kAccel + ((Math.abs(targetVelocity) > 30) ? Math.signum(targetVelocity): 0) * kStatic;
    }

    public void update() {
        speed = sensors.getShooterVelocity() / ticksPerRadian * radius;
        TelemetryUtil.packet.put("shootPow", feedForward());
        motorPriorities.get(5).setTargetPower(feedForward());
    }
}