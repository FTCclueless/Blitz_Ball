package org.firstinspires.ftc.teamcode.opmodes.testing.valueYoinkin;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.aim.Aim;
import org.firstinspires.ftc.teamcode.subsystems.aim.Shooter;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

import java.sql.Array;
import java.util.ArrayList;

@Config
@TeleOp
public class ShooterKinematicYoink extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        robot.aim.state = Aim.State.MANUAL_AIM;
        Shooter shooter = robot.shooter;
        double sum = 0;
        int count = 0;
        ArrayList<Double> vels = new ArrayList<Double>();

        ArrayList<Double> coolPow = new ArrayList<>();
        ArrayList<Double> coolVel = new ArrayList<>();
        ArrayList<Double> coolAvg = new ArrayList<>();

        boolean pastRight = false;
        boolean pastLeft = false;

        boolean shooting = false;

        boolean shotted = true;


        waitForStart();

        while (!isStopRequested() && count < 21) {
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
                shooting = true;
            }
            else if (gamepad1.right_trigger <= 0.3) {
                pastLeft = false;
            }

            double pow = shooter.maxVelocity / 20.0 * count;
            shooter.setTargetVel(pow);
            double vel = robot.sensors.getShooterVelocity() / shooter.ticksPerRadian / shooter.gearRatio / shooter.maxVelocity;
            TelemetryUtil.packet.put("shooterVel", vel);


            double avg = sum / vels.size();
            if (shooting) {
                if (avg - vel > 5) {
                    shotted = true;
                    coolPow.add(pow);
                    coolVel.add(vel);
                }
                if (shotted && avg - vel < 5) {
                    shooting = false;
                }
            }
            if (!shooting) {
                vels.add(vel);
                sum += vel;
                TelemetryUtil.packet.put("average", avg);
            }

            if (count == 20) {
                for (int i = 0; i < coolPow.size(); i++) {
                    Log.e(coolAvg.get(i) + "", "pow: " + coolPow.get(i) + "vel: " + coolVel.get(i));
                }
                count++;
            }
        }
    }
}
