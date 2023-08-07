package org.firstinspires.ftc.teamcode.subsystems.aim;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.MotorPriority;
import org.firstinspires.ftc.teamcode.utils.MyServo;
import org.firstinspires.ftc.teamcode.utils.PID;
import org.firstinspires.ftc.teamcode.utils.TrapezoidMotionProfile;

import java.util.ArrayList;

@Config
public class Shooter {
    DcMotorEx shooter;
    Sensors sensors;

    ArrayList<MotorPriority> motorPriorities;

    private final double shooterGearRatio = 46.0/9.0;
    private final double shooterTicksPerRadian = 145.090909091 / shooterGearRatio / 2/Math.PI;
    private final double powPerVel = 0; // TODO
    private final double kStatic = 0; // TODO
    public double shooterMaxPower = 0;

    public double shooterTargetPower;
    public double shooterCurrentPower;
    public double shooterErrorPower;

    public Shooter(HardwareMap hardwareMap, ArrayList<MotorPriority> motorPriorities, Sensors sensors) {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        this.sensors = sensors;
        this.motorPriorities = motorPriorities;
        motorPriorities.add(new MotorPriority(shooter, 3, 5));
        shooterCurrentPower = 0;
    }

    public void setTargetVel(double velocity) {
        this.shooterTargetPower = velocity * powPerVel + kStatic;
    }

    public void update() {
        // TODO fixme
        /*shooterPID.getOut(shooterTargetVelocity - shooterCurrentVelocity);
        shooterErrorPower = shooterTargetPower-shooterCurrentPower;*/

       // motorPriorities.get(5).setTargetPower(shooterTargetPower);
    }
}