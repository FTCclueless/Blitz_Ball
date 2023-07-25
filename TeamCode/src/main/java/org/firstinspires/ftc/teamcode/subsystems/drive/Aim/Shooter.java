package org.firstinspires.ftc.teamcode.subsystems.drive.Aim;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.MotorPriority;
import org.firstinspires.ftc.teamcode.utils.PID;

import java.util.ArrayList;

public class Shooter {
    DcMotorEx shooter;


    ArrayList<MotorPriority> motorPriorities;

    double shooterTargetPower;
    double shooterTargetAngle;

    double shooterCurrentPower;
    double shooterCurrentAngle;

    double shooterErrorPower;
    double shooterErrorAngle;

    public static PID shooterPID = new PID(1,0,0);


    public Shooter(HardwareMap hardwareMap, ArrayList<MotorPriority> motorPriorities) {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        this.motorPriorities = motorPriorities;

        motorPriorities.add(new MotorPriority(shooter, 3, 5));


        shooterCurrentAngle = 0;
        shooterCurrentPower = 0;


    }

    public void setTargetValues(double power, double angle) {
        this.shooterTargetPower = power;
        this.shooterTargetAngle = angle;

    }

    public void update() {
        shooterErrorAngle = shooterTargetAngle-shooterCurrentAngle;
        shooterErrorPower = shooterTargetPower-shooterCurrentPower;

        motorPriorities.get(5).setTargetPower(shooterErrorPower);
    }
}
