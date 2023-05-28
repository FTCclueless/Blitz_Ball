package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;

public class DriveCommand extends CommandBase {
    private final Drivetrain drivetrain;
    private final Gamepad gamepad;

    public DriveCommand (Drivetrain drivetrain, Gamepad gamepad) {
        this.drivetrain = drivetrain;
        this.gamepad = gamepad;

        addRequirements(drivetrain);
    }

    @Override
    public void execute () {
        drivetrain.drive(gamepad);
    }
}
