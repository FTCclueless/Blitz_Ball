package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.drive.aim.Turret;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

@Autonomous
public class TurretValueYoinker extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx turretMotor = hardwareMap.get(DcMotorEx.class, "turret");

        double maxVel = 0;
        double maxAccel = 0;
        double time = 0;
        double lastTime = 0;
        double vel = 0;
        double lastVel = 0;
        double counter = 0;
        double sum = 0;
        FtcDashboard dash = FtcDashboard.getInstance();
        TelemetryPacket p = new TelemetryPacket();

        waitForStart();

        for (int i = 0; i < 1000; i++) {
            p.put("velocity", 0);
            p.put("acceleration", 0);
            p.put("avg accel", 0);
            dash.sendTelemetryPacket(p);
            sleep(20);
        }

        turretMotor.setPower(1);

        while (!isStopRequested()) {
            counter ++;
            time = System.currentTimeMillis();
            vel = turretMotor.getVelocity(AngleUnit.RADIANS);
            double accel =  (vel-lastVel)/ (time-lastTime);
            sum += accel;
            p.put("velocity", vel);
            p.put("acceleration", accel);
            p.put("avg accel", sum / counter);
            dash.sendTelemetryPacket(p);
            // System.out.println("***** accel: " + accel + " " + "avg: " + sum/counter + " time: " + (time-lastTime));

            lastTime = time;
            lastVel = vel;
            sleep(50);


            }



        }
    }

