package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.MotorPriority;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;

import java.util.ArrayList;

public class Intake {
    PriorityMotor intake;
    HardwareQueue hardwareQueue;

    boolean previousButton = true;
    boolean intakeOn = false;
    boolean reverseDir = false; // regular direction is reverse
    double speed = 1;

    public Intake(HardwareMap hardwareMap, HardwareQueue hardwareQueue) {
        this.hardwareQueue = hardwareQueue;
        intake = new PriorityMotor(
            hardwareMap.get(DcMotorEx.class, "intake"),
            "intake",
            3, 5
        );
        intake.motor[0].setDirection(DcMotorSimple.Direction.REVERSE);
        intake.motor[0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        hardwareQueue.addDevice(intake);

    }

    public void turnOn() {
        intake.setTargetPower(speed);
        intakeOn = true;
    }
    public void reverseDirection(){

        if(!reverseDir) {
            intake.motor[0].setDirection(DcMotorSimple.Direction.REVERSE);
            reverseDir = true;
        }
        else {
            intake.motor[0].setDirection(DcMotorSimple.Direction.FORWARD);
            reverseDir =  false;
        }
    }

    public void turnOff()
    {
        intake.setTargetPower(0);
        intakeOn = false;
    }

    public void changeIntakeSpeed (double requestedSpeed) {
        speed = requestedSpeed;
    }

    public void intakeTeleOp(Gamepad gamepad)
    {
        TelemetryUtil.packet.put("leftTrigger", gamepad.left_trigger);
        TelemetryUtil.packet.put("previous", previousButton);
        TelemetryUtil.packet.put("intakeon", intakeOn);
        Log.e("intakeOn", intakeOn + "");

        if((gamepad.left_trigger >= .3) && previousButton) {
            previousButton = false;
            intakeOn = !intakeOn;
        }
        if(gamepad.left_trigger <= .3){
            previousButton = true;
        }
        if(intakeOn){
            turnOn();
        }
        else{
            turnOff();
        }
    }
}
