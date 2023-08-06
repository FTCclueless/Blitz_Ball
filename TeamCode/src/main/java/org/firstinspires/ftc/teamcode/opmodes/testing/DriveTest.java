package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.aim.Aim;
import org.firstinspires.ftc.teamcode.subsystems.aim.AimState;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

@TeleOp
public class DriveTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        Drivetrain drivetrain = robot.drivetrain;
        Aim aim = robot.aim;
        aim.aimState = AimState.MANUAL_AIM;

        waitForStart();

        while (!isStopRequested()) {
            drivetrain.drive(gamepad1);

            TelemetryUtil.packet.put("turret", Math.toDegrees(aim.turret.currentAngle));
            robot.update();
        }
    }
}
