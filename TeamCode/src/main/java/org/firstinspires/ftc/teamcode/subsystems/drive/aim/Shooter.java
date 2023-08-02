package org.firstinspires.ftc.teamcode.subsystems.drive.aim;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.MotorPriority;
import org.firstinspires.ftc.teamcode.utils.MyServo;
import org.firstinspires.ftc.teamcode.utils.PID;

import java.util.ArrayList;

public class Shooter {
    DcMotorEx shooter;
    MyServo leftShooterHood;
    MyServo rightShooterHood;
    Sensors sensors;


    ArrayList<MotorPriority> motorPriorities;

    double shooterGearRatio = 46.0/9.0;
    double shooterTicksPerRadian = 145.090909091 / shooterGearRatio;
    public double shooterMaxPower = 0;

    public double shooterTargetPower;
    public double shooterCurrentPower;
    public double shooterErrorPower;

    public static PID shooterPID = new PID(1,0,0);


    public Shooter(HardwareMap hardwareMap, ArrayList<MotorPriority> motorPriorities, Sensors sensors) {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        this.sensors = sensors;

        this.motorPriorities = motorPriorities;

        motorPriorities.add(new MotorPriority(shooter, 3, 5));


        shooterCurrentPower = 0;


    }

    public void setTargetVel(double power) {
        this.shooterTargetPower = power;

    }



    public void update() {
        shooterErrorPower = shooterTargetPower-shooterCurrentPower;

        motorPriorities.get(5).setTargetPower(shooterTargetPower);
    }
}