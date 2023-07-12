package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

@Autonomous
public class IMUtester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        BHI260IMU imu = hardwareMap.get(BHI260IMU.class, "imu");
        BHI260IMU.Parameters params = new IMU.Parameters(new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        ));
        imu.initialize(params);

        waitForStart();
        double past = 0;
        double sum = 0;
        double loops = 0;

        while (opModeIsActive()) {
            double start = System.nanoTime();
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            double end = System.nanoTime();
            loops ++;
            sum += ((start - end)/1.0E9);

            System.out.println("yaw " + orientation.getYaw(AngleUnit.DEGREES) + " " + sum/loops);
            past = System.currentTimeMillis();

        }
    }
}
