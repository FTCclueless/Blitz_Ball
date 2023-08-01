package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.Servo;

public class MyServo {
    double axonPositionPerAngle = 0.2; //temp angle
    double basePos = 0;
    String type;
    boolean isReversed;

    double currentPos;

    Servo servo;
    public MyServo(Servo servo, String type, double basePos, boolean reversed) {
        this.servo = servo;
        this.type = type;
        this.basePos = basePos;
        this.isReversed = reversed;
    }

    public void setAngle(double angle) {
        if (type == "axon") {
            servo.setPosition(angle * axonPositionPerAngle + basePos);
            currentPos = angle;
        }
    }
}
