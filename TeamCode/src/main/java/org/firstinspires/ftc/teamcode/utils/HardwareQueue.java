package org.firstinspires.ftc.teamcode.utils;
// THIS CLASS JANK

import java.util.Hashtable;

interface Functor {
    Object func(Object... args);
}

class FunctorPriority {
    private final long requestTime;
    private final HardwareBus bus;
    private final double individualWeight;
    protected final Object[] args;

    /**
     *
     * @param requestTime Nanos
     * @param bus
     * @param individualWeight 0-1 weight factor
     */
    public FunctorPriority(long requestTime, HardwareBus bus, double individualWeight, Object... args) {
        this.args = args;
        this.requestTime = requestTime;
        this.bus = bus;
        this.individualWeight = individualWeight;
    }

    public double getPriorityValue() {
        return (System.nanoTime() - requestTime) * bus.getWeight() * individualWeight;
    }
}

enum HardwareBus {
    I2C(1),
    MOTOR(1),
    ANALOG(1),
    DIGITAL(1),
    SERVO(1);

    private double weight;

    HardwareBus(double weight) {
        this.weight = weight;
    }

    public double getWeight() {
        return weight;
    }
}

public class HardwareQueue {
    Hashtable<Functor, FunctorPriority>[] functors = new Hashtable[HardwareBus.values().length];

    public HardwareQueue() {
        for (int i = 0; i < HardwareBus.values().length; i++) {
            functors[i] = new Hashtable<>();
        }
    }

    public void addRequest(Functor functor, HardwareBus bus, double individualWeight, Object... args) {
        // The least jank thing about this entire class
        int i = 0;
        for (HardwareBus b : HardwareBus.values()) {
            if (b == bus) {
                break;
            }
            i++;
        }

        functors[i].put(
            functor,
            new FunctorPriority(System.nanoTime(), bus, individualWeight, args)
        );
    }

    /*public void addRequestVoid(VoidFunctor functor, HardwareBus bus, Object... args) {
        // TODO For the jank
    }*/

    public void update(long targetNanos) {
        long start = System.nanoTime();
        while (System.nanoTime() - start < targetNanos) {
            for (Hashtable<Functor, FunctorPriority> ht : functors) {
                // Get the largest priority
                Functor largest = ht.keys().nextElement();
                for (Functor key : ht.keySet()) {
                    if (ht.get(key).getPriorityValue() > ht.get(largest).getPriorityValue()) {
                        largest = key;
                    }
                }
                largest.func(ht.get(largest).args);
            }
        }
    }
}
