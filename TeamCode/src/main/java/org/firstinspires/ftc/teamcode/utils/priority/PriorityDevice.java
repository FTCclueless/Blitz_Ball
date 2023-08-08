package org.firstinspires.ftc.teamcode.utils.priority;

public abstract class PriorityDevice {
    protected final double basePriority, priorityScale, lastUpdateTime;

    public PriorityDevice(double basePriority, double priorityScale) {
        this.basePriority = basePriority;
        this.priorityScale = priorityScale;
        lastUpdateTime = System.currentTimeMillis();
    }

    protected abstract double getPriority(double timeRemaining);

    protected abstract void update();
}
