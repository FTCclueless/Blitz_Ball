package org.firstinspires.ftc.teamcode.utils.priority;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class PriorityMotor extends PriorityDevice{
    double lastPower = 0;
    public double power = 0;
    public DcMotorEx[] motor; // if the subsystem has multiple motors (i.e. slides)

    public PriorityMotor(DcMotorEx motor, double basePriority, double priorityScale) {
        super(basePriority, priorityScale);
        this.motor = new DcMotorEx[] {motor};
    }

    public PriorityMotor(DcMotorEx[] motor, double basePriority, double priorityScale) {
        super(basePriority, priorityScale);
        this.motor = motor;
    }

    public void setTargetPower(double power) {
        this.power = power;
    }

    @Override
    protected double getPriority(double timeRemaining) {
        if (power-lastPower == 0) {
            lastUpdateTime = System.nanoTime() / 1000.0;
            return 0;
        }

        if (timeRemaining * 1000.0 <= 1.6 * (motor.length-1) + 0.8) {
            return 0;
        }

        return basePriority + Math.abs(power-lastPower) + (System.nanoTime() / 1000.0 - lastUpdateTime) * priorityScale;
    }

    @Override
    protected void update() {
        for (int i = 0; i < motor.length; i ++) {
            motor[i].setPower(power);
        }
        lastUpdateTime = System.currentTimeMillis();
        lastPower = power;
    }
}
