package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.drive.aim.Turret;
import org.firstinspires.ftc.teamcode.utils.AngleUtil;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

@Autonomous(group = "tests")
@Config
public class TurretPIDTuner extends LinearOpMode {
    public static double targetAngle = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        Turret.pidEnabled = true;

        while (!isStopRequested()) {
            TelemetryUtil.packet.put("Current Angle", AngleUtil.clipAngle(robot.turret.currentAngle));
            TelemetryUtil.packet.put("Target Angle", AngleUtil.clipAngle(robot.turret.targetAngle));
            TelemetryUtil.packet.put("Error Angle", AngleUtil.clipAngle(robot.turret.targetAngle - robot.turret.currentAngle));
            robot.turret.setTargetAngle(Math.toRadians(targetAngle));
            robot.update();
        }
    }
}
