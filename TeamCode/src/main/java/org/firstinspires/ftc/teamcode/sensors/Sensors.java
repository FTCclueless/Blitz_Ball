package org.firstinspires.ftc.teamcode.sensors;

import android.util.Log;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.utils.MotorPriority;

import java.util.ArrayList;

public class Sensors {
    LynxModule controlHub, expansionHub;
    private ArrayList<MotorPriority> motorPriorities;

    private IMU imu;
    private int[] odometry = new int[2];

    public Sensors (HardwareMap hardwareMap, ArrayList<MotorPriority> motorPriorities) {
        this.motorPriorities = motorPriorities;

        initHubs(hardwareMap);

        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters params = new IMU.Parameters(
            new RevHubOrientationOnRobot(new Orientation(
                AxesReference.INTRINSIC,
                AxesOrder.XYZ,
                AngleUnit.RADIANS,
                0, 0, 0,
                0 // Apparently unused
            ))
        );
        imu.initialize(params);
    }

    private void initHubs(HardwareMap hardwareMap) {
        try {
            controlHub = hardwareMap.get(LynxModule.class, "Control Hub");
            controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
            expansionHub = hardwareMap.get(LynxModule.class, "Expansion Hub");
            expansionHub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        } catch (RuntimeException e) {
            throw new RuntimeException("One or more of the REV hubs could not be found. More info: " + e);
        }
    }

    public void update() {
        updateControlHub();
        updateTelemetry();
    }

    private void updateControlHub() {
        try {
            odometry[0] = motorPriorities.get(0).motor[0].getCurrentPosition(); // left
            odometry[1] = motorPriorities.get(1).motor[0].getCurrentPosition(); // right
        }
        catch (Exception e) {
            Log.e("******* Error due to ", e.getClass().getName());
            e.printStackTrace();
            Log.e("******* fail", "control hub failed");
        }
    }

    public void updateTelemetry() {}

    public int[] getOdometry() {
        return odometry;
    }

    public IMU getImu() {
        return imu;
    }

    public double getIMUAngle() {
        Orientation robotOrientation;

        robotOrientation = imu.getRobotOrientation(
                AxesReference.INTRINSIC,
                AxesOrder.XYZ,
                AngleUnit.RADIANS
        );

        return normalizeAngle(robotOrientation.firstAngle);
    }

    double normalizedHeading = 0.0;
    double lastImuHeading = 0.0;

    public double normalizeAngle(double imuHeading) {
        double deltaHeading = imuHeading - lastImuHeading;
        while (Math.abs(deltaHeading) > Math.PI){
            deltaHeading -= Math.PI * 2 * Math.signum(deltaHeading);
        }
        lastImuHeading = imuHeading;
        normalizedHeading += deltaHeading;
        return normalizedHeading;
    }
}
