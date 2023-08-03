package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.drivers.REVColorSensorV3;

import java.util.Arrays;

@Autonomous(group = "tests", name = "Test Color Sensor V3")
public class TestREVColorSensorV3 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RevColorSensorV3 colorSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");
        /*REVColorSensorV3 colorSensor = hardwareMap.get(REVColorSensorV3.class, "colorSensor");

        waitForStart();

        REVColorSensorV3.ControlRequest request = new REVColorSensorV3.ControlRequest()
            .enableFlag(REVColorSensorV3.ControlFlag.RGB_ENABLED)
            .enableFlag(REVColorSensorV3.ControlFlag.LIGHT_SENSOR_ENABLED)
            .enableFlag(REVColorSensorV3.ControlFlag.PROX_SENSOR_ENABLED);

        colorSensor.sendControlRequest(request); // Enable RGB mode*/

        FtcDashboard dashboard = FtcDashboard.getInstance();

        waitForStart();

        while (opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();
            Canvas canvas = packet.fieldOverlay();

            long start = System.currentTimeMillis();
            int val = colorSensor.argb();
            long end = System.currentTimeMillis();
            System.out.println(end - start + " monkey");
            /*packet.put("RGB", Arrays.toString(color));
            for (int i = 0; i < color.length; i++) {
                color[i] *= 255;
            }
            canvas.setFill(String.format("#%02x%02x%02x", (int) color[0], (int) color[1], (int) color[2]));
            canvas.fillRect(-72, -72, 144, 144);
            dashboard.sendTelemetryPacket(packet);*/
        }
    }
}
