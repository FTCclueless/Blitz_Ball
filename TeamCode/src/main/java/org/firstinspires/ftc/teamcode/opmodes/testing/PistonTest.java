package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.aim.Aim;

@Config
@TeleOp
public class PistonTest extends LinearOpMode {
    public static double angle = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        robot.aim.setState(Aim.State.OFF);
        Transfer transfer = robot.aim.transfer;
        transfer.state = Transfer.State.HOLD;

        waitForStart();

        while (!isStopRequested()) {
            transfer.setPistonValue(angle);
            RobotLog.e(robot.sensors.getPistonPos() + "");
            robot.update();
        }
    }
}
