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

        while (opModeIsActive()) {
            Orientation orientation = imu.getRobotOrientation(
                AxesReference.INTRINSIC,
                AxesOrder.XYZ,
                AngleUnit.DEGREES
            );
            telemetry.addData("yaw", orientation.firstAngle + " " + System.currentTimeMillis());
            telemetry.update();
        }
    }
}
