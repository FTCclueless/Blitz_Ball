package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@TeleOp
public class IntakeTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        Intake intake = robot.intake;

        waitForStart();

        while (!isStopRequested()) {
            intake.intakeTeleOp(gamepad1);
            robot.update();
        }
    }
}
