package org.firstinspires.ftc.teamcode.subsystems.drive;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;

public class Intake {
    DcMotorEx intake;
    ArrayList<DcMotorEx> motorPriorities;


    public Intake(HardwareMap hardwareMap, ArrayList<DcMotorEx> motorPriorities) {
        hardwareMap.get(DcMotorEx.class, "intake");
        this.motorPriorities = motorPriorities;
    }
}
