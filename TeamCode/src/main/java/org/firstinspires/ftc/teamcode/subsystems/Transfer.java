package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drivers.REVColorSensorV3;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityCRServo;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;

import java.util.ArrayList;

public class Transfer {
    public enum State {
        EJECT,
        SHOOT,
        READ
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
    private final double pistonRetract = 0; // TOOD
    private final double pistonThresh = 0.1;
    private double pistonTargetPos = 0;

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

    public void ejectBall() {
        pistonTargetPos = pistonExtend;
    }


    public void update() {



        switch (state) {
            case READ:
                boolean oldState = beamBreakState;
                beamBreakState = beamBreak.getState();
                if (beamBreakState && !oldState) {
                    /*if (System.currentTimeMillis() > colorSensorLastUpdate + 200 && newBall) {
                    colorSensorLastUpdate = System.currentTimeMillis();

                    // Not my best work...
                    int[] rgb = colorSensorV3.readLSRGBRAW();
                    boolean breaken = false;
                    for (int i = 0; i < rgb.length; i++) {
                        if (rgb[i] < yellowRGBLow[i] || rgb[i] > yellowRGBHigh[i]) {
                            breaken = true;
                            break;
                        }
                    }*/
                }





                /*if (breaken) {
                    balls.add(Ball.YELLOW);
                    newBall = false;
                    return;
                }

                for (int i = 0; i < rgb.length; i++) {
                    if (rgb[i] < whiteRGBLow[i] && rgb[i] > whiteRGBHigh[i]) {
                        breaken = true;
                        break;
                    }
                }

                if (breaken) {
                    balls.add(Ball.WHITE);
                    newBall = false;
                    return;
                }*/

        // TODO piston pid (not ready yet mechanically)

        /*
        if (Math.abs(pistonPos - pistonExtend) > pistonThresh)
            pistonTargetPos = pistonRetract;
         */


        // Color sensor adding new balls




    }
}