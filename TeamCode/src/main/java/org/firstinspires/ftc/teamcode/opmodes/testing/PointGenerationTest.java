package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drive.Spline;
import org.firstinspires.ftc.teamcode.utils.Pose2d;

@Autonomous(group = "Auto")
public class PointGenerationTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap);
        Drivetrain drivetrain = robot.drivetrain;

        drivetrain.setPoseEstimate(new Pose2d(0, 0, 0));
        Spline spline = new Spline(new Pose2d(0, 0, 0), 8)
                .addSpline(new Pose2d(48, 0, 0))
                .addSpline(new Pose2d(48, 20, Math.toRadians(90)));;

        waitForStart();
        while (!isStopRequested()) {
            telemetry.addData("spline.points.size()", spline.poses.size());
            telemetry.update();
        }
    }
}
