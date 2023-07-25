package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.drive.Aim.Turret;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;

public class TurretTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        Drivetrain drivetrain = robot.drivetrain;
        Turret turret =  robot.turret;

        waitForStart();

        while(!isStopRequested()) {
            drivetrain.drive(gamepad1);
            turret.move(gamepad1);
            robot.update();
        }


    }
}
