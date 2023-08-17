package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
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
        Intake intake = robot.intake;
        robot.aim.state = Aim.State.AUTO_AIM; // aim in auto-aim
        // add transfer code here if needed
        Double lastButton = Double.valueOf(0);


        robot.aim.setTarget(1,30,30, Ball.YELLOW);
        robot.aim.setMainTarget(1);// change later when auto aim is working fully

        Boolean intakeOn = false;

        // TODO: a init statement to start the robot and hood at position 0.

        while(!isStopRequested()){
            // driver a
            drivetrain.drive(gamepad1);

            if(gamepad1.cross && lastButton != 1)
            {
                lastButton = Double.valueOf(1);
                intakeOn = !intakeOn;
                if(intakeOn == true){
                    intake.turnOn();
                }
                else{
                    intake.turnOff();
                }
            }

            if(gamepad1.circle && lastButton != 2){
                lastButton = Double.valueOf(2);
                intake.reverseDirection();
            }

            robot.update();




    }
}
