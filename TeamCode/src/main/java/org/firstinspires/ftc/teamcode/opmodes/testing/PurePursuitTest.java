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

        drivetrain.setPoseEstimate(new Pose2d(0, 0, 0));
        Spline spline = new Spline(drivetrain.getPoseEstimate(), 12)
            .addSpline(new Pose2d(48, 48, Math.PI / 2));

        waitForStart();
        robot.followSpline(spline, this);

        /*drivetrain.setPoseEstimate(new Pose(0, 0, 0));
        Pose midPoint = new Pose(52,20,Math.toRadians(90));
        Spline spline = new Spline(new Pose(0, 0, 0))
                .addPoint(new Pose(midPoint.x - 8, 0, 0))
                .addPoint(midPoint)
                .mustGoToPoint()
                .setReversed(true)
                .addPoint(new Pose(midPoint.x + 8,0,0))
                .addPoint(new Pose(midPoint.x * 2.0,0,0));

        waitForStart();
        robot.followSpline(spline, this);*/
    }
}
