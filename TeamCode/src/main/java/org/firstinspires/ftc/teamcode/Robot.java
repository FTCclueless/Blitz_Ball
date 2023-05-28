package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.utils.MotorPriority;

import java.util.ArrayList;

public class Robot {
    HardwareMap hardwareMap;

    Sensors sensors;
    Drivetrain drive;

    public ArrayList<MotorPriority> motorPriorities = new ArrayList<>();

    public Robot (HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        sensors = new Sensors(hardwareMap);
        drive = new Drivetrain(hardwareMap, motorPriorities, sensors);
    }

    public void update() {

    }
}
