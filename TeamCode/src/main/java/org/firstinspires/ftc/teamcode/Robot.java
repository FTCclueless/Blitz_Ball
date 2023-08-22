package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.utils.Globals.GET_LOOP_TIME;
import static org.firstinspires.ftc.teamcode.utils.Globals.START_LOOP;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.aim.Aim;
import org.firstinspires.ftc.teamcode.subsystems.aim.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.aim.Turret;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drive.Spline;
import org.firstinspires.ftc.teamcode.utils.DashboardUtil;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;

public class Robot {
    HardwareMap hardwareMap;

    public Sensors sensors;
    public Drivetrain drivetrain;
    public Turret turret;
    public Shooter shooter;
    public Aim aim;
    public Intake intake;

    public HardwareQueue hardwareQueue = new HardwareQueue();

    public Robot (HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        TelemetryUtil.setup();

        sensors = new Sensors(hardwareMap, hardwareQueue);
        drivetrain = new Drivetrain(hardwareMap, hardwareQueue, sensors);
        aim = new Aim(hardwareMap, hardwareQueue, sensors);
        turret = aim.turret;
        shooter = aim.shooter;
        intake = new Intake(hardwareMap, hardwareQueue);
    }

    public void update() {
        START_LOOP();
        updateSubsystems();
        updateTelemetry();
    }

    public void teleop(Gamepad gamepad1, Gamepad gamepad2) {
        START_LOOP();
        drivetrain.drive(gamepad1);
        intake.intakeTeleOp(gamepad1);
        aim.update();

        hardwareQueue.update();
    }

    private void updateSubsystems() {
        sensors.update();
        drivetrain.update();
        aim.update();

        hardwareQueue.update();
    }

    public void updateTelemetry() {
        TelemetryUtil.packet.put("Loop Time", GET_LOOP_TIME());

        updateDashboard();

        TelemetryUtil.sendTelemetry();
    }

    public void updateDashboard() {
        Canvas fieldOverlay = TelemetryUtil.packet.fieldOverlay();
        Pose2d poseEstimate = drivetrain.getPoseEstimate();

        DashboardUtil.drawRobot(fieldOverlay, poseEstimate);
        DashboardUtil.drawSampledPath(fieldOverlay, drivetrain.getCurrentPath());
    }

    public void followSplineWithTimer(Spline trajectory, LinearOpMode opMode, long startTime) {
        drivetrain.setCurrentPath(trajectory);
        while(drivetrain.isBusy() && (opMode.opModeIsActive() || (System.currentTimeMillis() - startTime >= 29500 && System.currentTimeMillis() - startTime <= 30800))) {
            update();
        }
    }

    public void followSpline(Spline spline, LinearOpMode opMode) {
        drivetrain.setCurrentPath(spline);
        while(drivetrain.isBusy() && opMode.opModeIsActive()) {
            update();
        }
    }


}
