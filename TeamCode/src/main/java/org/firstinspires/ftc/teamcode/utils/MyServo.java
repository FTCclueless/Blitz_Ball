package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.Servo;

public class MyServo {
    double axonPositionPerAngle = 0.2; //temp angle
    double axonZero = 0;
    String type;
    boolean isReversed;

    Servo servo;
    public MyServo(Servo servo, String type, boolean reversed) {
        this.servo = servo;
        this.type = type;
        this.isReversed = reversed;
    }

    public void setAngle(double angle) {
        if (type == "axon") {
            servo.setPosition(angle * axonPositionPerAngle + axonZero);
        }
    }
}
