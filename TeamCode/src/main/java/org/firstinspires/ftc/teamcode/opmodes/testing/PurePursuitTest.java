package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drive.Spline;
import org.firstinspires.ftc.teamcode.utils.Pose2d;

@Autonomous(group = "Auto")
public class PurePursuitTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap);
        Drivetrain drivetrain = robot.drivetrain;

        double garbageValue = -20;
        drivetrain.setPoseEstimate(new Pose2d(garbageValue, 0, 0));
        Pose2d midPoint = new Pose2d(52 + garbageValue,30,Math.toRadians(90));
        Spline spline1 = new Spline(drivetrain.getPoseEstimate(), 2)
            .addPoint(new Pose2d(midPoint.x - 16, 0, 0))
            .addPoint(new Pose2d(midPoint.x, midPoint.y, midPoint.heading));
        Spline spline2 = new Spline(midPoint, 2)
            .setReversed(true)
            .addPoint(new Pose2d(midPoint.x + 16,0,0))
            .addPoint(new Pose2d(midPoint.x * 2.0,0,0));

        waitForStart();
        robot.followSpline(spline1, this);
        robot.followSpline(spline2, this);

        while (!isStopRequested()) {
            robot.update();
        }
    }
}
