package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.drive.aim.Hood;

@TeleOp
@Config
public class LinkageTest extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);

        Hood hood = robot.aim.hood;
    }
}
