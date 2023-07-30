package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.drive.aim.Turret;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drive.aim.TurretState;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

@Config
@TeleOp(group = "tests")
public class TurretTest extends LinearOpMode {
    double x = 0;

    public static double target = 0.08;




    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        Drivetrain drivetrain = robot.drivetrain;
        Turret turret =  robot.turret;
        turret.turretState = TurretState.AUTOAIM;
        Turret.pidEnabled = false;

        waitForStart();

        while(!isStopRequested()) {
            // drivetrain.drive(gamepad1);


            robot.update();
            turret.setTargetAngle(target);


            TelemetryUtil.packet.put("current (rad)", (turret.currentAngle));
            TelemetryUtil.packet.put("velocity", robot.sensors.getTurretVelocity() / turret.ticksPerRadian);


        }


    }
}
