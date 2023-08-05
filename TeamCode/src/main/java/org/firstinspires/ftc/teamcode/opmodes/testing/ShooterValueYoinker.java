package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.aim.Shooter;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

@TeleOp
@Config
public class ShooterValueYoinker extends LinearOpMode {
    public static double target = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        Robot robot = new Robot(hardwareMap);
        Shooter shooter = robot.shooter;

        while (!isStopRequested()) {
            robot.motorPriorities.get(5).setTargetPower(target);
            robot.update();
            TelemetryUtil.packet.put("error", shooter.shooterErrorPower);
            TelemetryUtil.packet.put("current", shooter.shooterCurrentPower);
        }
    }
}
