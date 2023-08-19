package org.firstinspires.ftc.teamcode.opmodes.testing;

import android.renderscript.RenderScript;
import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.subsystems.aim.Aim;
import org.firstinspires.ftc.teamcode.subsystems.aim.Turret;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;

@Config
@TeleOp
public class DrivetrainFinder extends LinearOpMode {
    public static double target = 0.5;
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        Drivetrain drivetrain = robot.drivetrain;
        robot.aim.state = Aim.State.MANUAL_AIM;
        robot.aim.turret.state = Turret.State.OFF;
        Sensors sensors = robot.sensors;

        waitForStart();

        while (!isStopRequested()) {
            robot.update();
            ((PriorityMotor)robot.hardwareQueue.getDevice("leftFront")).setTargetPower(target);
            Log.e("targetPow", "" + ((PriorityMotor)robot.hardwareQueue.getDevice("leftRear")).getPower());

            TelemetryUtil.packet.put("pow", ((PriorityMotor)robot.hardwareQueue.getDevice("leftRear")).motor[0].getPower());
        }
    }
}
