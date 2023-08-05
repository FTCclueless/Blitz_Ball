package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.aim.AimState;
import org.firstinspires.ftc.teamcode.subsystems.aim.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.aim.Turret;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

@Config
@TeleOp(group = "tests")
public class AimTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        robot.aim.aimState = AimState.AUTO_AIM;
        Turret turret = robot.turret;
        Shooter shooter = robot.shooter;
        robot.aim.setTarget1(30,30);
        robot.aim.setMainTarget(1);

        waitForStart();
        while (!isStopRequested()) {
            robot.update();
            TelemetryUtil.packet.put("targetAngle", turret.targetAngle);
            TelemetryUtil.packet.put("errorAngle", turret.errorAngle);
            TelemetryUtil.packet.put("currentAngle",turret.currentAngle);
        }
    }
}
