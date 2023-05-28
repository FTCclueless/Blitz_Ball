package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;

public class DriveCommand extends CommandBase {
    private final Drivetrain drive;
    private final Gamepad gamepad;

    public DriveCommand (Drivetrain drive, Gamepad gamepad) {
        this.drive = drive;
        this.gamepad = gamepad;

        addRequirements(drive);
    }

    @Override
    public void execute () {
        double forward = -0.4*Math.tan(((gamepad.left_stick_y * -1 ) / 0.85));
        double left = -0.4*(Math.tan(gamepad.left_stick_x / 0.85)) * 0.8;
        double turn = gamepad.right_stick_x*0.9;

        double p1 = forward+left+turn;
        double p2 = forward-left+turn;
        double p3 = forward+left-turn;
        double p4 = forward-left-turn;
        drive.setMotorPowers(p1, p2, p3, p4);
    }
}
