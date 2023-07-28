package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.drive.aim.Turret;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

@Autonomous(group = "tests")
public class TurretTest extends LinearOpMode {
    double x = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        Drivetrain drivetrain = robot.drivetrain;
        Turret turret =  robot.turret;

        Turret.pidEnabled = false;

        waitForStart();

        while(!isStopRequested()) {
            // drivetrain.drive(gamepad1);

            drivetrain.update();
            turret.update();
            robot.update();


        }


    }
}
