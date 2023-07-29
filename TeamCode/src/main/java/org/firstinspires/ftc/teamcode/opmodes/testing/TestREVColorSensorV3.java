package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.drivers.REVColorSensorV3;

@Autonomous(group = "tests", name = "Test Color Sensor V3")
public class TestREVColorSensorV3 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        REVColorSensorV3 colorSensor = hardwareMap.get(REVColorSensorV3.class, "colorSensor");

        waitForStart();

        REVColorSensorV3.ControlRequest request = new REVColorSensorV3.ControlRequest()
            .enableFlag(REVColorSensorV3.ControlFlag.RGB_ENABLED);

        colorSensor.sendControlRequest(request); // Enable RGB mode

        while (opModeIsActive()) {

        }
    }
}
