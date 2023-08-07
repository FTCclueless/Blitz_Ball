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
    public static double target = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        Robot robot = new Robot(hardwareMap);
        Shooter shooter = robot.shooter;
        Sensors sensors = new Sensors(hardwareMap, robot.motorPriorities);
        robot.aim.aimState = AimState.MANUAL_AIM;
        double counter = 0;
        double sumPow = 0;
        double sumPow2 = 0;
        double sumVel = 0;
        double sumVel2 = 0;
        double sumProduct = 0;

        while (!isStopRequested() && counter < 10000) {
            counter ++;
            robot.update();
            double vel = sensors.getShooterVelocity() / 4.51798735686;
            double pow = counter/10000;

            robot.motorPriorities.get(5).setTargetPower(counter/10000);

            TelemetryUtil.packet.put("power", pow);

            sumPow += pow;
            sumPow2 += pow*pow;
            sumVel += vel;
            sumVel2 += vel*vel;
            sumProduct += vel*pow;


        }

        double yIntercept = (sumPow*sumVel2 - sumVel*sumProduct) / (counter * sumVel2 - Math.pow(sumVel, 2));
        double slope = (counter * sumProduct - sumVel*sumPow) / (counter*sumVel2 - Math.pow(sumVel,2));
        Log.e("yIntercept", "" + yIntercept);
        Log.e("slope", "" + slope);
    }
}
