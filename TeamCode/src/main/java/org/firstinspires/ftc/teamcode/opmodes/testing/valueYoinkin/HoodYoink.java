package org.firstinspires.ftc.teamcode.opmodes.testing.valueYoinkin;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.aim.Aim;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityServo;

@Config
@TeleOp
public class HoodYoink extends LinearOpMode {
    public static double p = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        robot.aim.setState(Aim.State.OFF);

        waitForStart();

        while (opModeIsActive()) {
            ((PriorityServo) robot.hardwareQueue.getDevice("shooterHood")).servo.setPosition(p);
            robot.update();
        }
    }
}