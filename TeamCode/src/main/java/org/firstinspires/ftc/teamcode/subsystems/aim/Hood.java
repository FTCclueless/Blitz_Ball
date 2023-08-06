package org.firstinspires.ftc.teamcode.subsystems.aim;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.utils.MyServo;

public class Hood {
    public final MyServo shooterHood;

    private final double linkageLength = 0; // TODO find r3
    private final double leverLength = 0; // TODO find r2
    private final double hoodLength = 0; // TODO find r1
    private final double sX = 0;
    private final double sY = 0;
    private double angle;
    double maxAngle = Math.toRadians(70); //TODO find
    double minAngle = Math.toRadians(15); //TODO find

    public Hood(HardwareMap hardwareMap) {
        // FIXME: min and max have not been calculated yet please write a test program someone
        // TODO also uhh do motorpriorities
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
        angle = Math.min(Math.max(minAngle, angle), maxAngle);

        // https://www.desmos.com/calculator/wkseaeitcf ee oo monkey see do hehee
        double mpx = hoodLength * Math.cos(angle);
        double mpy = hoodLength * Math.sin(angle);
        double d = mpx - sX;
        double f = Math.pow(linkageLength, 2) - Math.pow(d, 2) - Math.pow(leverLength, 2) + Math.pow(mpy, 2) - Math.pow(sY, 2);
        double a1 = Math.pow(2 * mpy - 2 * sY, 2) + 4 * Math.pow(d, 2);
        double b1 = -2 * f * (2 * mpy - 2 * sY) - 8 * Math.pow(d, 2) * mpy;
        double c1 = Math.pow(f, 2) - 4 * Math.pow(d, 2) * Math.pow(leverLength, 2) + 4 * Math.pow(d, 2) * Math.pow(mpy, 2);
        double y3 = (-b1 + Math.sqrt(Math.pow(b1, 2) - 4 * a1 * c1)) / (2 * a1);
        double b2 = -2 * sX;
        double c2 = Math.pow(sX, 2) - Math.pow(linkageLength, 2) + Math.pow(y3 - sX, 2);
        double x3a = (-b2 + Math.sqrt(Math.pow(b2, 2) - 4 * c2)) / 2;
        double n = Math.sqrt(Math.pow(x3a - mpx, 2) + Math.pow(y3 - mpy, 2));
        double l = Math.signum(Math.log(n / (leverLength - 10.0e-6)));
        double x3 = (-b2 + l * Math.sqrt(Math.pow(b2, 2) - 4 * c2)) / 2;
        double t2 = Math.atan2(y3 - sY, x3 - sX);

        shooterHood.setAngle(t2);

        this.angle = t2;
    }

    public double getAngle() {
        return shooterHood.getAngle();
    }

    public double getTarget() {
        return this.angle;
    }
}
