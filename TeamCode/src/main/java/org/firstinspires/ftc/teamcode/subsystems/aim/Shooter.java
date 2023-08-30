package org.firstinspires.ftc.teamcode.subsystems.aim;

import android.util.Log;

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
    public enum State {
        AUTO_AIM,
        OFF
    }
    PriorityMotor shooter;
    Sensors sensors;

    HardwareQueue hardwareQueue;

    public State state = State.AUTO_AIM;




    public final double gearRatio = 46.0/16.0;
    public final double radius = 3.25;
    public final double ticksPerRadian = 537.689839572 / gearRatio / (2.0*Math.PI);
    private final double kStatic = 0.07806370549956637; //0.2194966041039161; // TODO
    private final double powPerVel = 0.0030768805766372164; //0.002355662527748475; // TODO
    public static double kAccel = 2;
    public double maxVelocity = (1.0 - kStatic) / powPerVel;
    public double shooterMaxPower = 0;
    private double speed = 0;
    public static double lowpassWeight = 0.98;

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
        switch (state) {
            case AUTO_AIM:
                TelemetryUtil.packet.put("shootPow", feedForward());
                shooter.setTargetPower(feedForward());
                break;
            case OFF:
                break;
        }
    }
}