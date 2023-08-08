package org.firstinspires.ftc.teamcode.opmodes.testing;

import android.hardware.Sensor;
import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.subsystems.aim.AimState;
import org.firstinspires.ftc.teamcode.subsystems.aim.Shooter;
import org.firstinspires.ftc.teamcode.utils.MotorPriority;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

import java.util.ArrayList;

@TeleOp
@Config
public class ShooterValueYoinker extends LinearOpMode {
    public static int tests = 1;
    public static int samples = 10000;

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        Robot robot = new Robot(hardwareMap);
        Shooter shooter = robot.shooter;
        Sensors sensors = new Sensors(hardwareMap, robot.motorPriorities);
        robot.aim.aimState = AimState.MANUAL_AIM;

        waitForStart();
        double avgYint = 0;
        double avgSlope = 0;

        double counter = 0;
        double sumPow = 0;
        double sumPow2 = 0;
        double sumVel = 0;
        double sumVel2 = 0;
        double sumProduct = 0;

        for (int j = 0; j <= samples; j++) {
            robot.update();
            double vel = shooter.getSpeed();
            double pow = (double) j / samples;

            TelemetryUtil.packet.put("power", pow);

            if (vel > 1) {
                sumPow += pow;
                sumPow2 += pow * pow;
                sumVel += vel;
                sumVel2 += vel * vel;
                sumProduct += vel * pow;
                counter ++;
            }
            robot.motorPriorities.get(5).setTargetPower((double) j / samples);

        }

        double yIntercept = (sumPow * sumVel2 - sumVel * sumProduct) / (counter * sumVel2 - Math.pow(sumVel, 2));
        double slope = (counter * sumProduct - sumVel * sumPow) / (counter * sumVel2 - Math.pow(sumVel, 2));
        avgYint += yIntercept;
        avgSlope += slope;
        Log.e("yIntercept", "" + yIntercept);
        Log.e("slope", "" + slope);

        // Stop it
        robot.motorPriorities.get(5).setTargetPower(0);

        }
}
