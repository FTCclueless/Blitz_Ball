package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.aim.Aim;
import org.firstinspires.ftc.teamcode.subsystems.aim.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.aim.Turret;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

@TeleOp
public class DriveTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        Drivetrain drivetrain = robot.drivetrain;
        Aim aim = robot.aim;
        aim.state = Aim.State.MANUAL_AIM;
        aim.turret.state = Turret.State.OFF;
        aim.shooter.state = Shooter.State.OFF;

        waitForStart();

        while (!isStopRequested()) {
            drivetrain.drive(gamepad1);

            TelemetryUtil.packet.put("leftStickY", gamepad1.left_stick_y);
            TelemetryUtil.packet.put("rightStickX", gamepad1.right_stick_x);

            TelemetryUtil.packet.put("turret", Math.toDegrees(aim.turret.currentAngle));
            TelemetryUtil.packet.put("turretVel", robot.aim.turret.turretVelocity);
            robot.update();
        }
    }
}
