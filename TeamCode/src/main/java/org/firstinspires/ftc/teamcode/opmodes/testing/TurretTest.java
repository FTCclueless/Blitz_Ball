package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.aim.Aim;
import org.firstinspires.ftc.teamcode.subsystems.aim.Turret;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

@Config
@TeleOp(group = "tests")
public class TurretTest extends LinearOpMode {
    public static double target = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        Robot robot = new Robot(hardwareMap);
        Turret turret = robot.turret;
        turret.state = Turret.State.AUTOAIM;
        robot.aim.state = Aim.State.MANUAL_AIM;

        while (!isStopRequested()) {
            turret.setTargetAngle(Math.toRadians(target));
            TelemetryUtil.packet.put("error", turret.errorAngle);
            TelemetryUtil.packet.put("velocity", turret.turretVelocity);
            TelemetryUtil.packet.put("current", turret.currentAngle);
            robot.update();
        }

    }
}
