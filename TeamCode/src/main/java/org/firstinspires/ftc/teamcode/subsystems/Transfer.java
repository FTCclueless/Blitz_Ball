package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.PID;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityCRServo;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityServo;

public class Transfer {
    private final HardwareQueue hardwareQueue;
    private PriorityMotor liftMotor;
    private PriorityCRServo transferServo;
    private final Sensors sensors;
    public static PID pid = new PID(1, 0, 0);

    public enum State {
        MANUAL_CONTROL,
        PID_ENABLED
    }

    public double speed = 0.2;

    public Transfer(HardwareMap hardwareMap, HardwareQueue hardwareQueue, Sensors sensors) {
        DcMotorEx transferElevator = hardwareMap.get(DcMotorEx.class, "transferElevator");
        transferElevator.setDirection(DcMotorSimple.Direction.REVERSE);
        transferElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        transferElevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        CRServo transferWheel = hardwareMap.get(CRServo.class, "transferWheel");

        this.hardwareQueue = hardwareQueue;
        this.sensors = sensors;

        liftMotor = new PriorityMotor(
                transferElevator,
                "liftMotor",
                2,
                4);
        hardwareQueue.addDevice(liftMotor);

        transferServo = new PriorityCRServo(
                transferWheel,
                "transferServo",
                2,
                4);
    }

    public void turnOn() {
        transferServo.setTargetPower(0.2);
    }

    public void turnOff() {
        transferServo.setTargetPower(0);
    }


}