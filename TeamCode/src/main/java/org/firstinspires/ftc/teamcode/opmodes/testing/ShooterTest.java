package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.aim.Aim;
import org.firstinspires.ftc.teamcode.subsystems.aim.Shooter;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

@Config
@TeleOp
public class ShooterTest extends LinearOpMode {
    public static double velocity;
    public static boolean shoot = false;

    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap);
        Shooter shooter = robot.shooter;
        robot.aim.state = Aim.State.MANUAL_AIM;
        robot.aim.transfer.state = Transfer.State.READ_BEAMBREAK;
        robot.aim.transfer.turnOn();
        robot.intake.turnOn();

        waitForStart();

        while (!isStopRequested()) {
            shooter.setTargetVel(velocity);

            if (shoot) {
                robot.aim.transfer.shootBall();
                shoot = false;
            }

            TelemetryUtil.packet.put("Shooter Velocity", shooter.getSpeed());
            TelemetryUtil.packet.put("Target Velocity", shooter.getTargetVel());
            robot.update();
        }
    }
}
