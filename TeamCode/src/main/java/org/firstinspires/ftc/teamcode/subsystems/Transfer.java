package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drivers.REVColorSensorV3;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityCRServo;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityServo;

import java.util.ArrayList;

@Config
public class Transfer {
    public enum State {
        EJECT,
        SHOOT,
        READ_BEAMBREAK,
        READ_COLOR,
        HOLD // Hold all values for other subystems
    }

    private final HardwareQueue hardwareQueue;
    private final Sensors sensors;

    private PriorityMotor liftMotor;
    private final double pistonTickPerRadian = 85.5759958182;
    private double pistonShoot = 3.14;
    private double pistonHalf = 1.6;
    private double pistonRetract = 0; // TOOD
    public double pistonCurrent = 0;
    private static double pistonThresh = 0.1;
    /*public double pistonCurrent = 0;
    public static double pistonPowVel = 0.029475550925660274;
    public static double pistonStatic = 0.04604534417389545;
    private double pistonMaxVel = 0;
    public static double pistonSlowDown = 2;
    public static double pistonMargin = 0.5;*/

    private PriorityCRServo transferServo;

    public State state = State.READ_BEAMBREAK;

    private final REVColorSensorV3 colorSensorV3;
    private long colorSensorLastUpdate = System.currentTimeMillis();
    private long colorTime = 210;
    private final int[] whiteRGBLow = new int[]{0, 0, 0}; // TODO
    private final int[] whiteRGBHigh = new int[]{20, 330, 0}; // TODO
    private final int[] yellowRGBLow = new int[]{0, 0, 0}; // TODO
    private final int[] yellowRGBHigh = new int[]{330, 102, 3}; // TODO

    private final DigitalChannel beamBreak;
    private boolean beamBreakState = false;

    private final PriorityServo ejectServo;
    private final double ejectAngle = Math.toRadians(220); // TODO
    private final double unejectAngle = Math.toRadians(300); // TODO
    private double lastEjectTime = 0;
    private boolean ejecting = false;

    public Ball currentBall = Ball.EMPTY;
    private double ballRollTime = 400; // ms TODO

    public Transfer(HardwareMap hardwareMap, HardwareQueue hardwareQueue, Sensors sensors) {
        beamBreak = hardwareMap.get(DigitalChannel.class, "beamBreak");

        colorSensorV3 = hardwareMap.get(REVColorSensorV3.class, "colorSensor");
        REVColorSensorV3.ControlRequest controlRequest = new REVColorSensorV3.ControlRequest()
            .enableFlag(REVColorSensorV3.ControlFlag.RGB_ENABLED)
            .enableFlag(REVColorSensorV3.ControlFlag.LIGHT_SENSOR_ENABLED);
        colorSensorV3.sendControlRequest(controlRequest);
        colorSensorV3.configureLSRecording(REVColorSensorV3.LSResolution.TWENTY, REVColorSensorV3.LSMeasureRate.m200s);

        DcMotorEx transferElevator = hardwareMap.get(DcMotorEx.class, "transferElevator");
        transferElevator.setDirection(DcMotorSimple.Direction.REVERSE);
        transferElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        transferElevator.setTargetPosition(0);
        transferElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        CRServo transferWheel = hardwareMap.get(CRServo.class, "transferWheel");
        transferWheel.setDirection(DcMotorSimple.Direction.REVERSE);

        this.hardwareQueue = hardwareQueue;
        this.sensors = sensors;

        liftMotor = new PriorityMotor(
            transferElevator,
            "piston",
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
        hardwareQueue.addDevice(transferServo);

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
        hardwareQueue.addDevice(ejectServo);
    }


    public void turnOn() {
        transferServo.setTargetPower(0.2);
    }

    public void turnOff() {
        transferServo.setTargetPower(0);
    }


    public void shootBall() {
        state = State.SHOOT;
    }

    public void ejectBall() {
        state = State.EJECT;
    }

    public void setPistonValue(double angle) {
        liftMotor.motor[0].setPower(1);
        liftMotor.motor[0].setTargetPosition((int) (angle * pistonTickPerRadian));
        /*pistonCurrent = sensors.getPistonPos()/pistonTickPerRadian;
        ((PriorityMotor)hardwareQueue.getDevice("piston")).setTargetPower(angle-pistonCurrent);*/
    }

    /*public double pistonFeedForward(double target) {
        double error = target - pistonCurrent;
        double vel = error*(pistonMaxVel/pistonSlowDown);
        double pow = vel * pistonPowVel + pistonStatic;
        return pow;
    }*/

    private boolean rgbCompare(int[] rgb, int[] low, int[] high) {
        for (int i = 0; i < rgb.length; i++) {
            if (rgb[i] < low[i] || rgb[i] > high[i]) {
                return false;
            }
        }
        return true;
    }


    public void update() {
        pistonCurrent = sensors.getPistonPos() / pistonTickPerRadian;

        switch (state) {
            case READ_BEAMBREAK:
                // ((PriorityMotor)hardwareQueue.getDevice("liftMotor")).setTargetPower(pistonFeedForward(pistonRetract));
                boolean oldState = beamBreakState;
                beamBreakState = !beamBreak.getState();
                System.out.println(beamBreakState + " " + oldState);
                if (beamBreakState && !oldState) {
                    System.out.println("BALLS IN MY JAWS");
                    //state = State.READ_COLOR;
                }
                break;

            case READ_COLOR:
                if (System.currentTimeMillis() >= colorSensorLastUpdate + colorTime) {
                    if (colorSensorV3.lsNewData()) {
                        int[] rgb = colorSensorV3.readLSRGBRAW();

                        if (rgbCompare(rgb, yellowRGBLow, yellowRGBHigh)) {
                            currentBall = Ball.YELLOW;
                        } else if (rgbCompare(rgb, whiteRGBLow, whiteRGBHigh)) {
                            currentBall = Ball.WHITE;
                        }
                        colorTime = 210;
                    } else {
                        colorTime = 50;
                    }
                    colorSensorLastUpdate = System.currentTimeMillis();
                }
                break;

            case EJECT:
                currentBall = Ball.EMPTY;
                if (!ejecting) {
                    ejectServo.setTargetAngle(ejectAngle, 0.75);
                    setPistonValue(pistonHalf);
                    // ((PriorityMotor)hardwareQueue.getDevice("liftMotor")).setTargetPower(pistonFeedForward(pistolHalf));
                    lastEjectTime = System.currentTimeMillis();
                    TelemetryUtil.packet.put("ServoAngle", ejectServo.getCurrentAngle());
                    if (Math.abs(pistonHalf - pistonCurrent) < pistonThresh && ejectServo.getCurrentAngle() == ejectAngle) {
                        ejecting = true;
                    }
                } else {
                    if (System.currentTimeMillis() > lastEjectTime + ballRollTime ) {
                        ejectServo.setTargetAngle(unejectAngle, 0.75);
                        setPistonValue(pistonRetract);
                        // ((PriorityMotor)hardwareQueue.getDevice("liftMotor")).setTargetPower(pistonFeedForward(pistonRetract));
                        if (Math.abs(pistonRetract - pistonCurrent) < pistonThresh && ejectServo.getCurrentAngle() == unejectAngle) {
                            ejecting = false;
                            state = State.READ_BEAMBREAK;
                            setPistonValue(pistonRetract);
                        }
                    }
                }

                break;
            case SHOOT:
                currentBall = Ball.EMPTY;
                // ((PriorityMotor) hardwareQueue.getDevice("liftMotor")).setTargetPower(pistonFeedForward(pistonShoot));
                setPistonValue(pistonShoot);
                if (Math.abs(pistonShoot - pistonCurrent) < pistonThresh) {
                    state = State.READ_BEAMBREAK;
                    setPistonValue(pistonRetract);
                }
                break;

            case HOLD:
                break;
        }
    }
}