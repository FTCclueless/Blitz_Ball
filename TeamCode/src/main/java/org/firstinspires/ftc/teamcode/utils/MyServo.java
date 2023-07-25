package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.Servo;

public class MyServo {
    double axonPositionPerAngle;
    Servo servo;
    public MyServo(Servo servo) {
        this.servo = servo;
    }

    public void setAngle(double angle, String type) {
        if (type == "axon") {
            servo.setPosition(angle * axonPositionPerAngle);
        }
    }
}
