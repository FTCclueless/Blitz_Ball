package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Ball;
import org.firstinspires.ftc.teamcode.subsystems.aim.Aim;
import org.firstinspires.ftc.teamcode.subsystems.aim.Hood;
import org.firstinspires.ftc.teamcode.subsystems.aim.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.aim.Turret;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;

public class Teleop extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        //initialization
        Robot robot = new Robot(hardwareMap);
        Drivetrain drivetrain = robot.drivetrain;
        Hood hood = robot.aim.hood;
        Shooter shooter = robot.aim.shooter;
        Turret turret = robot.turret;
        robot.aim.state = Aim.State.AUTO_AIM; // aim in auto-aim
        // add transfer code here if needed


        robot.aim.setTarget(1,30,30, Ball.YELLOW);
        robot.aim.setMainTarget(1);// change later when auto aim is working fully

        //Is it a finite state machine? Can't find state in the robot class :-(

        // TODO: a init statement to start the robot and hood at position 0.

        while(!isStopRequested()){
            // driver a
            drivetrain.drive(gamepad1);



        }

    }
}
