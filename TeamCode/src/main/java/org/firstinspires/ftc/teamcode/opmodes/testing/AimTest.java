package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.drive.aim.AimState;
import org.firstinspires.ftc.teamcode.subsystems.drive.aim.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.drive.aim.Turret;

@Config
@TeleOp(group = "tests")
public class AimTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        robot.aim.aimState = AimState.AUTO_AIM;
        Turret turret = robot.turret;
        Shooter shooter = robot.shooter;

        waitForStart();
        while (!isStopRequested()) {
            robot.update();
        }
    }
}
