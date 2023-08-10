package org.firstinspires.ftc.teamcode.subsystems.aim;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.MotorPriority;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;

import java.util.ArrayList;

@Config
public class Shooter {
    PriorityMotor shooter;
    Sensors sensors;

    HardwareQueue hardwareQueue;



    private final double gearRatio = 46.0/9.0;
    private final double radius = 3.25;
    private final double ticksPerRadian = 145.090909091 / gearRatio / (2.0*Math.PI);
    private final double powPerVel = 0.00050180086; //0.002355662527748475; // TODO
    private final double kStatic = 0.176; //0.2194966041039161; // TODO
    public static double kAccel = 4;
    public double maxVelocity = (1.0 - kStatic) / powPerVel;
    public double shooterMaxPower = 0;
    private double speed = 0;
    public static double lowpassWeight = 0;

    public double targetVelocity;
    public double shooterCurrentPower;
    public double shooterErrorPower;

    public Shooter(HardwareMap hardwareMap, HardwareQueue hardwareQueue, Sensors sensors) {
        shooter = new PriorityMotor(
            hardwareMap.get(DcMotorEx.class, "shooter"),
            "shooter",
            3, 5
        );
        shooter.motor[0].setDirection(DcMotorEx.Direction.REVERSE);

        speed = sensors.getShooterVelocity() / ticksPerRadian * radius;
        this.sensors = sensors;
        this.hardwareQueue = hardwareQueue;
        hardwareQueue.addDevice(shooter);
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
        return targetVelocity * powPerVel + (targetVelocity - speed) / maxVelocity * kAccel + ((Math.abs(targetVelocity) > 20) ? Math.signum(targetVelocity): 0) * kStatic;
    }

    public void update() {
        speed = speed * lowpassWeight + (sensors.getShooterVelocity() / ticksPerRadian * radius) * (1 - lowpassWeight);
        TelemetryUtil.packet.put("shootPow", feedForward());
        shooter.setTargetPower(feedForward());
    }
}