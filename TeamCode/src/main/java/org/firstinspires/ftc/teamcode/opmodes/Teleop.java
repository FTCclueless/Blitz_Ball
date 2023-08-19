package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Ball;
import org.firstinspires.ftc.teamcode.subsystems.aim.Aim;
import org.firstinspires.ftc.teamcode.subsystems.aim.Hood;
import org.firstinspires.ftc.teamcode.subsystems.aim.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.aim.Turret;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;

@TeleOp
public class Teleop extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        //initialization
        Robot robot = new Robot(hardwareMap);
        robot.aim.setState(Aim.State.MANUAL_AIM);

        robot.aim.addTarget(30,30, Ball.YELLOW);
        robot.aim.setMainTarget(0);// change later when auto aim is working fully

        waitForStart();

        // TODO: a init statement to start the robot and hood at position 0.

        while(!isStopRequested()) {
            robot.teleop(gamepad1, gamepad2);
        }
    }
}
