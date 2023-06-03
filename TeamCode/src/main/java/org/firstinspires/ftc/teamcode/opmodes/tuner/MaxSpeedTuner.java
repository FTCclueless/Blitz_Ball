package org.firstinspires.ftc.teamcode.opmodes.tuner;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;

@TeleOp
public class MaxSpeedTuner extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        Drivetrain drivetrain = robot.drivetrain;
        double maxVelocity = 0.0;

        waitForStart();

        while (!isStopRequested()) {
            drivetrain.drive(gamepad1);

            if (drivetrain.localizer.relCurrentVel.x > maxVelocity) {
                maxVelocity = drivetrain.localizer.relCurrentVel.x;
            }

            robot.update();

            telemetry.addData("maxVelocity", maxVelocity + "");
            telemetry.update();
        }
    }
}
