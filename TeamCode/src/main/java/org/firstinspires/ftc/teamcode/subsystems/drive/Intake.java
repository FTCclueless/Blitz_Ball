package org.firstinspires.ftc.teamcode.subsystems.drive;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.MotorPriority;

import java.util.ArrayList;

public class Intake {
    DcMotorEx intake;
    ArrayList<MotorPriority> motorPriorities;

    boolean previousButton = true;

    boolean intakeOn = false;

    double maxSpeed = .5;

    public Intake(HardwareMap hardwareMap, ArrayList<MotorPriority> motorPriorities) {
        this.motorPriorities = motorPriorities;
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        motorPriorities.add((new MotorPriority (intake, 3, 5)));

    }

    public void turnOnIntake() {
        motorPriorities.get(6).setTargetPower(maxSpeed);
        intakeOn = true;
    }
    public void turnOffIntake()
    {
        motorPriorities.get(6).setTargetPower(maxSpeed);
        intakeOn = false;
    }

    public void IntakeTeleOp(Gamepad gamepad)
    {

        if((gamepad.left_trigger >.3) && previousButton == true) {
            previousButton = false;
            intakeOn = !intakeOn;
        }
        if(gamepad.left_trigger<.3){
            previousButton = true;
        }
        if(intakeOn == true){
            turnOnIntake();
        }
        else{
            turnOffIntake();
        }
    }
}
