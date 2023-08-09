package org.firstinspires.ftc.teamcode.utils.priority;

public abstract class PriorityDevice {
    protected final double basePriority, priorityScale;
    protected double lastUpdateTime;

    public PriorityDevice(double basePriority, double priorityScale) {
        this.basePriority = basePriority;
        this.priorityScale = priorityScale;
        lastUpdateTime = System.nanoTime() / 1000.0;
    }

    protected abstract double getPriority(double timeRemaining);

    protected abstract void update();
}
