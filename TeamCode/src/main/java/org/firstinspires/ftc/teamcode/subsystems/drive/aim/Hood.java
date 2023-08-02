package org.firstinspires.ftc.teamcode.subsystems.drive.aim;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.MyServo;

public class Hood {
    MyServo leftShooterHood;
    MyServo rightShooterHood;

    double currentAngle = 0;
    double maxAngle = Math.toRadians(70); //TODO find
    double minAngle = Math.toRadians(15); //TODO find

    double leftZero = 0; //TODO find these values
    double rightZero = 0; //TODO find these values

    public Hood(HardwareMap hardwareMap) {
        leftShooterHood = new MyServo(hardwareMap.get(Servo.class, "leftShooterHood"),"axon", leftZero, true);
        rightShooterHood = new MyServo(hardwareMap.get(Servo.class, "rightShooterHood"), "axon", rightZero, false);
    }

    public void setAngle(double angle) {
        currentAngle = angle;
        leftShooterHood.setAngle(angle);
        rightShooterHood.setAngle(angle);
    }


}
