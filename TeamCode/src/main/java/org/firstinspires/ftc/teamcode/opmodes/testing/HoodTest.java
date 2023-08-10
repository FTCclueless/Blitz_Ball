package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.aim.Aim;
import org.firstinspires.ftc.teamcode.subsystems.aim.Hood;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

@Config
@TeleOp
public class HoodTest extends LinearOpMode {
    public static double target = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        Hood hood = robot.aim.hood;
        Aim aim = robot.aim;
        aim.state = Aim.State.MANUAL_AIM;
        while (!isStopRequested()) {
            hood.setAngle(target);
            TelemetryUtil.packet.put("right", hood.getAngle());
            TelemetryUtil.packet.put("target", target);
            robot.update();
        }
    }
}
