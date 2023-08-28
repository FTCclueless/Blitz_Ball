package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.aim.Aim;

@Config
@TeleOp
public class TransferTest extends LinearOpMode {
    public static Transfer.State state = Transfer.State.READ_BEAMBREAK;
    public static boolean updateState = true;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        robot.aim.setState(Aim.State.OFF);
        robot.aim.transfer.state = state;

        waitForStart();

        while (opModeIsActive()) {
            if (updateState) {
                robot.aim.transfer.state = state;
                updateState = false;
            }
            robot.update();
        }
    }
}
