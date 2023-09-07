package org.firstinspires.ftc.teamcode.subsystems.aim;

import android.util.Log;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.utils.MyServo;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityCRServo;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityServo;

public class Hood {
    public final PriorityServo shooterHood;
    private double angle;
    public final double minAngle = Math.toRadians(0);
    public final double maxAngle = Math.toRadians(24);

    public Hood(HardwareMap hardwareMap, HardwareQueue hardwareQueue) {
        shooterHood = new PriorityServo(
            hardwareMap.get(Servo.class, "leftHood"),
            "shooterHood",
            PriorityServo.ServoType.SPEED,
            .75,
            0,
            1,
            0.25,
            false,
            2,
            4
        );
        hardwareQueue.addDevice(shooterHood);
        this.angle = minAngle;
    }

    public void setAngle(double angle) {
        angle -= Math.toRadians(50);
        if (angle < minAngle || angle > maxAngle) {
            Log.e("shooterAngle out of range", "e");
        }
        angle = Math.min(Math.max(minAngle, angle), maxAngle);
        this.angle = angle;

        // angle = Math.PI / 2 - angle;

        shooterHood.setTargetAngle(angle, 0.75);
    }

    public double getAngle() {
        return angle;
    }
}
