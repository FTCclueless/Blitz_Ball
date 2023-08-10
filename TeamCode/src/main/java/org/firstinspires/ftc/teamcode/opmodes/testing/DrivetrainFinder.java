package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.subsystems.aim.Aim;
import org.firstinspires.ftc.teamcode.subsystems.aim.Turret;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;

@TeleOp
public class DrivetrainFinder extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        Drivetrain drivetrain = robot.drivetrain;
        robot.aim.state = Aim.State.MANUAL_AIM;
        robot.aim.turret.state = Turret.State.OFF;
        Sensors sensors = robot.sensors;

        waitForStart();

        while (!isStopRequested()) {
            robot.update();
            TelemetryUtil.packet.put("angle", sensors.getTurretAngle() / robot.aim.turret.ticksPerRadian);
            TelemetryUtil.packet.put("turretAngle", robot.aim.turret.currentAngle);
        }
    }
}
