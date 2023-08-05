package org.firstinspires.ftc.teamcode.subsystems.aim;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.MyServo;

public class Hood {
    public MyServo leftShooterHood;
    public MyServo rightShooterHood;

    public double rightAngle = 0;
    double maxAngle = Math.toRadians(70); //TODO find
    double minAngle = Math.toRadians(15); //TODO find

    double leftZero = 0; //TODO find these values
    double rightZero = 0; //TODO find these values

    public Hood(HardwareMap hardwareMap) {
        leftShooterHood = new MyServo(
                hardwareMap.get(Servo.class, "leftHood"),
                MyServo.ServoType.SPEED,
                .75,
                0,
                1,
                0,
                false
        );
        rightShooterHood = new MyServo(
                hardwareMap.get(Servo.class, "rightHood"),
                MyServo.ServoType.TORQUE,
                .75,
                0,
                1,
                0,
                true
        );
    }
    public void setAngle(double angle) {
        rightAngle = angle;
        leftShooterHood.setAngle(angle);
        rightShooterHood.setAngle(angle);
    }

    public double getLeft() {
        return leftShooterHood.getAngle();
    }

    public double getRight() {
        return rightShooterHood.getAngle();
    }


}
