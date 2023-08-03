package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.Servo;

public class MyServo {
    public enum ServoType {
        TORQUE(0.2162104887, Math.toRadians(60) / 0.25),
        SPEED(0.2162104887, Math.toRadians(60) / 0.11),
        SUPER_SPEED(0.2162104887, Math.toRadians(60) / 0.055),
        AMAZON(0.2122065908, Math.toRadians(60) / 0.13),
        PRO_MODELER(0.32698, Math.toRadians(60) / 0.139),
        JX(0.3183098862, Math.toRadians(60) / 0.12);

        public final double positionPerRadian;
        public final double speed;

        ServoType(double positionPerRadian, double speed) {
            this.positionPerRadian = positionPerRadian;
            this.speed = speed;
        }
    }

    public final Servo servo;
    private final double min, max, basePos;
    private double currentPos, currentAngle, positionPerRadian, speed;
    public double offset = 0;
    private long lastUpdateTime = System.nanoTime();

    public MyServo(Servo servo, ServoType type, double loadMultiplier, double min, double max, double basePos, boolean reversed) {
        this.servo = servo;
        this.positionPerRadian = type.positionPerRadian;
        this.speed = type.speed;
        this.speed *= loadMultiplier;
        this.min = min;
        this.max = max;
        this.basePos = basePos;
        currentPos = basePos;
        currentAngle = basePos / type.positionPerRadian;
        if (reversed) {
            positionPerRadian *= -1;
        }
    }

    public void setPosition(double targetPosition, double power) {
        targetPosition += offset;
        targetPosition = Math.max(Math.min(targetPosition, Math.max(min, max)), Math.min(min, max));
        double targetOrientation = targetPosition / positionPerRadian; // converts position to radian angle
        double error = targetOrientation - currentAngle;
        long currentTime = System.nanoTime();
        double time = (double) (currentTime - lastUpdateTime) / 1.0E9; // converts from nano to secs
        lastUpdateTime = currentTime;
        double update = Math.signum(error) * speed * power * time; // update is the distance (in radians) the servo has moved in a loop at the specified power
        currentAngle += update;
        if (Math.abs(update) >= Math.abs(error)) { // if setting servo position to update will cause the servo to go past it's target, set update = error
            currentAngle = targetPosition / positionPerRadian;
        }

        currentPos = currentAngle * positionPerRadian; // This converts the currentAngle (in radians) to a position value (between 0-1) and then adds the intercept
        if (power == 1.0) {
            servo.setPosition(targetPosition);
        } else {
            servo.setPosition(currentPos);
        }
    }

    public void setPosition(double position) {
        setPosition(position, 1.0);
    }

    public void setAngle(double angle, double power) {
        setPosition(angle * positionPerRadian * Math.signum(max - min) + basePos, power);
    }

    public void setAngle(double angle) {
        setPosition(angle * positionPerRadian * Math.signum(max - min) + basePos, 1.0);
    }

    public double getAngle() {
        return (currentAngle) + (offset - basePos) / positionPerRadian;
    }

    public double getCurrentPosition() {
        return currentPos;
    }

//    public double clipAngle (double angle) {
//        while (angle > 2*Math.PI) {
//            angle -= 2*Math.PI * 2.0;
//        }
//        while (angle < -2*Math.PI) {
//            angle += 2*Math.PI * 2.0;
//        }
//        return angle;
//    }
}