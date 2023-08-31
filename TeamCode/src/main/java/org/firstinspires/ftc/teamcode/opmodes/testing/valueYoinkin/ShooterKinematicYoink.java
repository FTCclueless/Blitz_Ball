package org.firstinspires.ftc.teamcode.opmodes.testing.valueYoinkin;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.aim.Aim;
import org.firstinspires.ftc.teamcode.subsystems.aim.Shooter;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityServo;

import java.sql.Array;
import java.util.ArrayList;

@Config
@TeleOp
public class ShooterKinematicYoink extends LinearOpMode {
    public static double hoodAngle = 0;
    public static double samples = 50;

    @Override
    public void runOpMode() throws InterruptedException {
        Log.e("CSVID", "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@");
        Robot robot = new Robot(hardwareMap);
        ((PriorityServo) robot.hardwareQueue.getDevice("ejectServo")).setTargetAngle(Math.toRadians(320), 0.75);
        robot.intake.turnOn();
        robot.aim.transfer.turnOn();
        robot.aim.state = Aim.State.MANUAL_AIM;
        robot.aim.shooter.state = Shooter.State.OFF;
        Shooter shooter = robot.shooter;
        int count = 0;
        Log.e("CSVID", "Power,Velocity,Battery Voltage,Hood Angle");

        boolean pastRight = false;
        boolean pastLeft = false;

        boolean recovering = true; // Need to go back to normal speed
        double minimum = Double.POSITIVE_INFINITY;
        long recoveryStart = System.currentTimeMillis();

        waitForStart();

        while (!isStopRequested() && count <= samples) {
            robot.aim.hood.setAngle(Math.toRadians(hoodAngle));

            if (!pastRight && gamepad1.right_trigger > 0.3) {
                pastRight = true;
                count ++;
            }
            else if (gamepad1.right_trigger <= 0.3) {
                pastRight = false;
            }

            if (!pastLeft && gamepad1.left_trigger > 0.3) {
                pastLeft = true;
                robot.aim.transfer.shootBall();
                recovering = true;
                recoveryStart = System.currentTimeMillis();
            }
            else if (gamepad1.right_trigger <= 0.3) {
                pastLeft = false;
            }

            double pow = count / samples;
            TelemetryUtil.packet.put("shooterPower", pow);
            shooter.shooter.setTargetPower(pow);
            double vel = robot.sensors.getShooterVelocity() / shooter.ticksPerRadian * shooter.radius;
            TelemetryUtil.packet.put("shooterVel", vel);

            /*if (gamepad1.a && !lastA) {
                double batteryVoltage  = Double.POSITIVE_INFINITY;
                for (VoltageSensor v : hardwareMap.voltageSensor) {
                    double voltage = v.getVoltage();
                    if (voltage > 0) {
                        batteryVoltage = Math.min(batteryVoltage, voltage);
                    }
                }

                CSVBuf += String.format("%f,%f,%f,%f\n", pow,  minimum, batteryVoltage, hoodAngle);
                recovering = true;
                recoveryStart = System.currentTimeMillis();
            }*/

            if (recovering) {
                gamepad1.rumble(1, 1, 500);

                if (System.currentTimeMillis() - recoveryStart > 1500) {
                    double batteryVoltage  = Double.POSITIVE_INFINITY;
                    for (VoltageSensor v : hardwareMap.voltageSensor) {
                        double voltage = v.getVoltage();
                        if (voltage > 0) {
                            batteryVoltage = Math.min(batteryVoltage, voltage);
                        }
                    }

                    Log.e("CSVID", String.format("%f,%f,%f,%f", pow,  minimum, batteryVoltage, hoodAngle));
                    recovering = false;
                    minimum = vel;
                }
            }
            minimum = Math.min(vel, minimum);


            robot.update();
        }
    }
}
