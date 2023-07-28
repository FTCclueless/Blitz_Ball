package org.firstinspires.ftc.teamcode.subsystems.drive;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.MotorPriority;

import java.util.ArrayList;

public class Intake {
    DcMotorEx intake;
    ArrayList<DcMotorEx> motorPriorities;

    boolean intakeState = true;

    double maxSpeed = 1.0;

    public Intake(HardwareMap hardwareMap, ArrayList<DcMotorEx> motorPriorities) {
        this.motorPriorities = motorPriorities;
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        motorPriorities.add((DcMotorEx) new MotorPriority (intake, 3, 5));

    }

    public void turnOnIntake() {
        intake.setPower(maxSpeed);
    }
    public void turnOffIntake()
    {
        intake.setPower(0);
    }

}
