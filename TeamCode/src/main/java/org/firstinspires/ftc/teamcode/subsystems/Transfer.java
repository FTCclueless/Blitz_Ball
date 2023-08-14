package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drivers.REVColorSensorV3;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityCRServo;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityServo;

import java.util.ArrayList;

public class Transfer {
    public enum State {
        EJECT,
        SHOOT,
        READ,
        MANUAL
    }

    private final HardwareQueue hardwareQueue;
    private PriorityMotor liftMotor;
    private PriorityCRServo transferServo;
    private final Sensors sensors;
    public State state = Transfer.State.READ;
    private final REVColorSensorV3 colorSensorV3;
    private final DigitalChannel beamBreak;
    private boolean beamBreakState = false;
    private boolean newBall = false;
    private long colorSensorLastUpdate = System.currentTimeMillis();
    private ArrayList<Ball> balls = new ArrayList<>();

    private final double pistonExtend = 0; // TODO
    private final double pistonHalf = 0; //todo
    private final double pistonRetract = 0; // TOOD
    private final double pistonThresh = 0.1;
    private double pistonTargetPos = 0;
    public double pistonCurrent = 0;
    public static double pistonPowVel = 0;
    public static double pistonStatic = 0;
    private double pistonMaxVel = 0;
    public static double pistonSlowDown = 0;
    public static double pistonMargin = 10;

    private Ball currentBall = Ball.EMPTY;
    private final PriorityServo ejectServo;

    // I SUCK AT VARIABLE NAMING
    private final double ejectAngle = 0; // TODO
    private final double unejectAngle = 0; // TODO
    private double lastEjectTime = 0;
    private boolean ejecting = false;
    private double ballRollTime = 0; // ms TODO

    private boolean shooting = false;

    // Bit confused on this part but whatever
    private final int[] whiteRGBLow = new int[]{0, 0, 0}; // TODO
    private final int[] whiteRGBHigh = new int[]{20, 330, 0}; // TODO
    private final int[] yellowRGBLow = new int[]{0, 0, 0}; // TODO
    private final int[] yellowRGBHigh = new int[]{330, 102, 3}; // TODO

    public double speed = 0.2;

    public Transfer(HardwareMap hardwareMap, HardwareQueue hardwareQueue, Sensors sensors) {
        beamBreak = hardwareMap.get(DigitalChannel.class, "beamBreak");

        colorSensorV3 = hardwareMap.get(REVColorSensorV3.class, "colorSensor");
        REVColorSensorV3.ControlRequest controlRequest = new REVColorSensorV3.ControlRequest()
            .enableFlag(REVColorSensorV3.ControlFlag.RGB_ENABLED);
        colorSensorV3.configureLSRecording(REVColorSensorV3.LSResolution.TWENTY, REVColorSensorV3.LSMeasureRate.m200s);

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
            4
        );
        hardwareQueue.addDevice(liftMotor);

        transferServo = new PriorityCRServo(
            transferWheel,
            "transferServo",
            2,
            4
        );

        ejectServo = new PriorityServo(
            hardwareMap.get(Servo.class, "eject"),
            "ejectServo",
            PriorityServo.ServoType.AXON_MINI,
            0.75,
            0, 1,
            0,
            false,
            2,
            4
        );
    }


    public void turnOn() {
        transferServo.setTargetPower(0.2);
    }

    public void turnOff() {
        transferServo.setTargetPower(0);
    }

    public void ejectBall() {
        pistonTargetPos = pistonExtend;
    }

    public double pistonFeedForward(double target) {
        double error = target - pistonCurrent;
        double vel = error*(pistonMaxVel/pistonSlowDown);
        double pow = vel * pistonPowVel + pistonStatic;
        return pow;
    }


    public void update() {



        switch (state) {
            case READ:
                ((PriorityMotor)hardwareQueue.getDevice("liftMotor")).setTargetPower(pistonFeedForward(pistonRetract));
                boolean oldState = beamBreakState;
                beamBreakState = beamBreak.getState();
                if (beamBreakState && !oldState) {
                    // TODO
                }

                break;
            case EJECT:
                if (!ejecting) {
                    ejectServo.setTargetAngle(ejectAngle, 0.75);
                    ((PriorityMotor)hardwareQueue.getDevice("liftMotor")).setTargetPower(pistonFeedForward(pistonHalf));
                    lastEjectTime = System.currentTimeMillis();
                    ejecting = true;
                } else {
                    if (System.currentTimeMillis() > lastEjectTime + ballRollTime ) {
                        ejecting = false;
                        state = State.READ;
                    }
                }

                break;
            case SHOOT:
                if (!shooting) {
                    ((PriorityMotor)hardwareQueue.getDevice("liftMotor")).setTargetPower(pistonFeedForward(pistonExtend));
                    if (Math.abs(pistonExtend - pistonCurrent) <  pistonMargin) {
                        shooting = true;
                    }
                }
                else {
                    ((PriorityMotor)hardwareQueue.getDevice("liftMotor")).setTargetPower(pistonFeedForward(pistonRetract));
                    // if (Math.abs()) LEFT OFF HERE
                }

                break;
        }
    }
}