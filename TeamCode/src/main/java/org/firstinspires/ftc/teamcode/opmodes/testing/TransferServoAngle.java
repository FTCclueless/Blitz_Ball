package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Ball;
import org.firstinspires.ftc.teamcode.subsystems.aim.Aim;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityServo;

@TeleOp
@Config
public class TransferServoAngle extends LinearOpMode {
    public static double angle = 180;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        PriorityServo servo = (PriorityServo) robot.hardwareQueue.getDevice("ejectServo");
        robot.aim.state = Aim.State.OFF;

        waitForStart();

        while (opModeIsActive()) {
            servo.setTargetAngle(Math.toRadians(angle), 0.1);
            TelemetryUtil.packet.put("TranServpos", servo.getCurrentAngle());
            robot.update();
        }
    }
}
