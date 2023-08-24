package org.firstinspires.ftc.teamcode.opmodes.testing.valueYoinkin;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drivers.REVColorSensorV3;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

import java.util.Arrays;

@TeleOp
@Config
public class ColorYoink extends LinearOpMode {
    public static boolean recording = false;
    int[] minColor = new int[] {Integer.MAX_VALUE, Integer.MAX_VALUE, Integer.MAX_VALUE};
    int[] maxColor = new int[] {0, 0, 0};

    @Override
    public void runOpMode() throws InterruptedException {
        REVColorSensorV3 colorSensorV3 = hardwareMap.get(REVColorSensorV3.class, "colorSensor");
        REVColorSensorV3.ControlRequest controlRequest = new REVColorSensorV3.ControlRequest()
                .enableFlag(REVColorSensorV3.ControlFlag.RGB_ENABLED);
        colorSensorV3.sendControlRequest(controlRequest);
        colorSensorV3.configureLSRecording(REVColorSensorV3.LSResolution.TWENTY, REVColorSensorV3.LSMeasureRate.m200s);

        waitForStart();

        while (opModeIsActive()) {
            if (recording) {
                int[] color = colorSensorV3.readLSRGBRAW();

                boolean min = false;
                boolean max = false;

                for (int i = 0; i < color.length; i++) {
                    if (color[i] > minColor[i]) {
                        min = true;
                    }
                    if (color[i] > maxColor[i]) {
                        max = true;
                    }
                }

                if (min) {
                    minColor = color;
                } else if (max) {
                    maxColor = color;
                }
            }
        }
        TelemetryUtil.packet.put("min", Arrays.toString(minColor));
        TelemetryUtil.packet.put("max", Arrays.toString(maxColor));
        TelemetryUtil.sendTelemetry();
    }
}
