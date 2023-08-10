package org.firstinspires.ftc.teamcode.opmodes.testing;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.subsystems.aim.Aim;
import org.firstinspires.ftc.teamcode.subsystems.aim.Shooter;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;

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
        Sensors sensors = new Sensors(hardwareMap, robot.hardwareQueue);
        robot.aim.state = Aim.State.MANUAL_AIM;

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
            ((PriorityMotor) robot.hardwareQueue.getDevice("shooter")).setTargetPower((double) j / samples);
        }

        double yIntercept = (sumPow * sumVel2 - sumVel * sumProduct) / (counter * sumVel2 - Math.pow(sumVel, 2));
        double slope = (counter * sumProduct - sumVel * sumPow) / (counter * sumVel2 - Math.pow(sumVel, 2));
        avgYint += yIntercept;
        avgSlope += slope;
        Log.e("yIntercept", "" + yIntercept);
        Log.e("slope", "" + slope);

        // Stop it
        ((PriorityMotor) robot.hardwareQueue.getDevice("shooter")).setTargetPower(0);

        while (Math.abs(shooter.getSpeed()) > 0.05) {
            robot.update();
        }
        // Try getting k static by doing a less cool method

        /*for (int i = 0; i < samples && opModeIsActive(); i++) {
            double pow = (double) i  / samples;
            robot.motorPriorities.get(5).setTargetPower(pow);
            TelemetryUtil.packet.put("pow", pow);

            if (Math.abs(shooter.getSpeed()) > 100) {
                Log.e("kStatic", pow + "");
                break;
            }
            robot.update();
        }*/

        }
}
