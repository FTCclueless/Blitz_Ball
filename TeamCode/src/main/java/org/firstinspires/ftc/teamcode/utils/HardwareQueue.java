package org.firstinspires.ftc.teamcode.utils;

import java.util.Hashtable;

interface Functor {
    Object func(Object... args);
}

enum HardwareBus {
    I2C,
    MOTOR,
    ANALOG,
    DIGITAL,
    SERVO
}

public class HardwareQueue {
    Hashtable<Functor, Object[]>[] functors = new Hashtable[HardwareBus.values().length];

    public HardwareQueue() {
        for (int i = 0; i < HardwareBus.values().length; i++) {
            functors[i] = new Hashtable<>();
        }
    }

    public void addRequest(Functor functor, HardwareBus bus, Object... args) {
        // The least jank thing about this entire class
        int i = 0;
        for (HardwareBus b : HardwareBus.values()) {
            if (b == bus) {
                break;
            }
            i++;
        }

        functors[i].put(functor, args);
    }

    /*public void addRequestVoid(VoidFunctor functor, HardwareBus bus, Object... args) {
        // TODO For the jank
    }*/

    public void update() {
        // Test
        addRequest((a) -> a, HardwareBus.SERVO, (Object) 19);
    }
}
