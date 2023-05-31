package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.utils.Globals.GET_LOOP_TIME;
import static org.firstinspires.ftc.teamcode.utils.Globals.LOOP_TIME;
import static org.firstinspires.ftc.teamcode.utils.Globals.START_LOOP;

import android.util.Log;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drive.Spline;
import org.firstinspires.ftc.teamcode.utils.DashboardUtil;
import org.firstinspires.ftc.teamcode.utils.MotorPriority;
import org.firstinspires.ftc.teamcode.utils.MyPose2d;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

import java.util.ArrayList;

public class Robot {
    HardwareMap hardwareMap;

    public Sensors sensors;
    public Drivetrain drivetrain;

    public ArrayList<MotorPriority> motorPriorities = new ArrayList<>();

    public Robot (HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        TelemetryUtil.setup();

        sensors = new Sensors(hardwareMap);
        drivetrain = new Drivetrain(hardwareMap, motorPriorities, sensors);
    }

    public void update() {
        START_LOOP();
        updateSubsystems();
    }

    private void updateSubsystems() {
        sensors.update();
        drivetrain.update();

        updateTelemetry();

        MotorPriority.updateMotors(motorPriorities);
        TelemetryUtil.sendTelemetry();
    }

    public void updateTelemetry() {
        TelemetryUtil.packet.put("Loop Time", GET_LOOP_TIME());

        updateDashboard();
    }

    public void updateDashboard() {
        Canvas fieldOverlay = TelemetryUtil.packet.fieldOverlay();
        MyPose2d poseEstimate = drivetrain.getPoseEstimate();

        TelemetryUtil.packet.put("x", poseEstimate.getX());
        TelemetryUtil.packet.put("y", poseEstimate.getY());
        TelemetryUtil.packet.put("heading (deg)", Math.toDegrees(poseEstimate.getHeading()));

        DashboardUtil.drawRobot(fieldOverlay, poseEstimate);
        DashboardUtil.drawSampledPath(fieldOverlay, drivetrain.currentSplineToFollow);
    }

    public void followSplineWithTimer(Spline trajectory, LinearOpMode opMode, long startTime) {
        drivetrain.setSpline(trajectory);
        while(drivetrain.isBusy() && (opMode.opModeIsActive() || (System.currentTimeMillis() - startTime >= 29500 && System.currentTimeMillis() - startTime <= 30800))) {
            update();
        }
    }

    public void followSpline(Spline spline, LinearOpMode opMode) {
        drivetrain.setSpline(spline);
        while(drivetrain.isBusy() && opMode.opModeIsActive()) {
            update();
        }
    }
}
