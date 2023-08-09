package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.MotorPriority;
import org.firstinspires.ftc.teamcode.utils.PID;

import java.util.ArrayList;

public class Transfer {
    private final ArrayList<MotorPriority> motorPriorities;
    private final Sensors sensors;
    public static PID pid = new PID(1, 0, 0);

    public enum State {
        MANUAL_CONTROL,
        PID_ENABLED
    }

    public Transfer(HardwareMap hardwareMap, ArrayList<MotorPriority> motorPriorities, Sensors sensors) {
        DcMotorEx transferElevator = hardwareMap.get(DcMotorEx.class, "transferElevator");
        transferElevator.setDirection(DcMotorSimple.Direction.REVERSE);
        transferElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        transferElevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorPriorities.add(new MotorPriority(transferElevator, 2, 4));

        this.motorPriorities = motorPriorities;
        this.sensors = sensors;
    }
}