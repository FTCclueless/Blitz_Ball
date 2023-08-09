package org.firstinspires.ftc.teamcode.utils.priority;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.MyServo;

public class PriorityServo extends PriorityDevice{
    Servo servo;
    double targetPos;
    public PriorityServo(Servo servo, double basePriority, double priorityScale) {
        super(basePriority, priorityScale);
        this.servo = servo;
    }

    public void setPos(double pos) {
        return; //TOdo do these
    }


    @Override
    protected double getPriority(double timeRemaining) {
        return 0.0;
    }

    @Override
    protected void update() {
        return;
    }
}
