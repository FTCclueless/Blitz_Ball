package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drive.Spline;
import org.firstinspires.ftc.teamcode.utils.MyPose2d;

@Autonomous(group = "Auto")
public class PurePursuitTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap);
        Drivetrain drivetrain = robot.drivetrain;

        drivetrain.setPoseEstimate(new MyPose2d(0, 0, 0));
        MyPose2d midPoint = new MyPose2d(52,20,Math.toRadians(90));
        Spline spline = new Spline(new MyPose2d(0, 0, 0))
                .addPoint(new MyPose2d(midPoint.x - 8, 0, 0))
                .addPoint(midPoint)
                .mustGoToPoint()
                .setReversed(true)
                .addPoint(new MyPose2d(midPoint.x + 8,0,0))
                .addPoint(new MyPose2d(midPoint.x * 2.0,0,0));

        waitForStart();
        robot.followSpline(spline, this);
    }
}
