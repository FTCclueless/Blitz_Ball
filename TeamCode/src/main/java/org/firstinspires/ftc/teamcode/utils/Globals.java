package org.firstinspires.ftc.teamcode.utils;

public class Globals {
    public static long LOOP_START = System.nanoTime();
    public static double LOOP_TIME = 0.0;
    public static double TRACK_WIDTH = 13.3750297033534346;
    public static double K_STATIC = 0;

    // subsystems enabled?
    public static boolean DRIVETRAIN_ENABLED = true;

    public static void START_LOOP() {
        LOOP_START = System.nanoTime();
    }

    public static double GET_LOOP_TIME() {
        LOOP_TIME = (System.nanoTime() - LOOP_START) / 1.0e9; // converts from nano secs to secs
        return LOOP_TIME;
    }
}
