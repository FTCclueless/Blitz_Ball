package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.utils.Globals.START_LOOP;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.utils.MotorPriority;

import java.util.ArrayList;

public class Robot {
    HardwareMap hardwareMap;

    public Sensors sensors;
    public Drivetrain drivetrain;

    public ArrayList<MotorPriority> motorPriorities = new ArrayList<>();

    public Robot (HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        sensors = new Sensors(hardwareMap);
        drivetrain = new Drivetrain(hardwareMap, motorPriorities, sensors);
    }

    public void update() {
        START_LOOP();
        updateSubsystems();
    }

    private void updateSubsystems() {
        sensors.update();
        drivetrain.update();

        MotorPriority.updateMotors(motorPriorities);
    }
}
