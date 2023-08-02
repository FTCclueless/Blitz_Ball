package org.firstinspires.ftc.teamcode.subsystems.drive;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.MotorPriority;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

import java.util.ArrayList;

public class Intake {
    DcMotorEx intake;
    ArrayList<MotorPriority> motorPriorities;

    boolean previousButton = true;

    boolean intakeOn = false;

    double maxSpeed = 1;

    public Intake(HardwareMap hardwareMap, ArrayList<MotorPriority> motorPriorities) {
        this.motorPriorities = motorPriorities;
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        motorPriorities.add((new MotorPriority (intake, 3, 5)));

    }

    public void turnOnIntake() {
        motorPriorities.get(6).setTargetPower(maxSpeed);
        intakeOn = true;
    }
    public void turnOffIntake()
    {
        motorPriorities.get(6).setTargetPower(0);
        intakeOn = false;
    }

    public void changeIntakeSpeed (double requestedSpeed) {
        maxSpeed = requestedSpeed;
    }

    public void IntakeTeleOp(Gamepad gamepad)
    {
        TelemetryUtil.packet.put("leftTrigger", gamepad.left_trigger);
        TelemetryUtil.packet.put("previous", previousButton);
        TelemetryUtil.packet.put("intakeon", intakeOn);
        Log.e("intakeOn", intakeOn + "");

        if((gamepad.left_trigger >= .3) && previousButton == true) {
            previousButton = false;
            intakeOn = !intakeOn;
        }
        if(gamepad.left_trigger <= .3){
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
