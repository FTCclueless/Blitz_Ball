package org.firstinspires.ftc.teamcode.subsystems.aim;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.utils.MyServo;

public class Hood {
    public final MyServo shooterHood;

    private final double linkageLength = 0; // TODO find
    private final double leverLength = 0; // TODO find
    private double angle;
    double maxAngle = Math.toRadians(70); //TODO find
    double minAngle = Math.toRadians(15); //TODO find

    public Hood(HardwareMap hardwareMap) {
        // FIXME: min and max have not been calculated yet please write a test program someone
        shooterHood = new MyServo(
                hardwareMap.get(Servo.class, "leftHood"),
                MyServo.ServoType.SPEED,
                .75,
                0,
                1,
                0,
                false
        );
        this.angle = minAngle;
    }
    public void setAngle(double angle) {
        if (angle < minAngle || angle > maxAngle) {
            RobotLog.ee("Hood", "PLACEHOLDER ERROR: angle set badly");
            angle = Math.min(Math.max(minAngle, angle), maxAngle);
        }

        this.angle = angle;
    }

    public double getAngle() {
        return angle;
    }
}
