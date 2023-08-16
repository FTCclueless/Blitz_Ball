package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityServo;

@TeleOp
@Config
public class TransferServoAngle extends LinearOpMode {
    public static double angle = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        PriorityServo servo = (PriorityServo) robot.hardwareQueue.getDevice("ejectServo");

        waitForStart();

        while (opModeIsActive()) {
            servo.setTargetAngle(angle, 0.75);
        }
    }
}
