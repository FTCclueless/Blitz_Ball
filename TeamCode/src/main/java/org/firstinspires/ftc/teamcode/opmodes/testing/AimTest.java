package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Ball;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.aim.Aim;
import org.firstinspires.ftc.teamcode.subsystems.aim.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.aim.Turret;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

@Config
@TeleOp(group = "tests")
public class AimTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        robot.aim.state = Aim.State.AUTO_AIM;
        Turret turret = robot.turret;
        Shooter shooter = robot.shooter;
        robot.aim.transfer.state = Transfer.State.READ_BEAMBREAK;
        robot.aim.transfer.turnOn();
        robot.intake.turnOn();

        robot.aim.addTarget(48,0, Ball.YELLOW);
        robot.aim.setMainTarget(0);
        boolean oldA = false;

        waitForStart();
        while (!isStopRequested()) {
            if (gamepad1.a && !oldA) {
                robot.aim.transfer.shootBall();
            }
            oldA = gamepad1.a;
            robot.update();
            TelemetryUtil.packet.put("targetAngle", turret.targetAngle);
            TelemetryUtil.packet.put("errorAngle", turret.errorAngle);
            TelemetryUtil.packet.put("currentAngle",turret.currentAngle);
            TelemetryUtil.packet.put("pistonPos", robot.sensors.getPistonPos());
        }
    }
}
