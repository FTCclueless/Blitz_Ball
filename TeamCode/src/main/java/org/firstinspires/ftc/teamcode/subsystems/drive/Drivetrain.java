package org.firstinspires.ftc.teamcode.subsystems.drive;

import static org.firstinspires.ftc.teamcode.utils.Globals.DRIVETRAIN_ENABLED;
import static org.firstinspires.ftc.teamcode.utils.Globals.MIN_MOTOR_POWER_TO_OVERCOME_FRICTION;
import static org.firstinspires.ftc.teamcode.utils.Globals.TRACK_WIDTH;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.subsystems.drive.localizers.TwoWheelLocalizer;
import org.firstinspires.ftc.teamcode.utils.MotorPriority;
import org.firstinspires.ftc.teamcode.utils.MyPose2d;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class Drivetrain {
    public DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private List<DcMotorEx> motors;

    private ArrayList<MotorPriority> motorPriorities;

    private TwoWheelLocalizer localizer;

    public Spline currentSplineToFollow = new Spline(new MyPose2d(0,0,0));

    public Drivetrain(HardwareMap hardwareMap, ArrayList<MotorPriority> motorPriorities, Sensors sensors) {
        this.motorPriorities = motorPriorities;

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (int i = 0; i < motors.size(); i ++) {
            MotorConfigurationType motorConfigurationType = motors.get(i).getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motors.get(i).setMotorType(motorConfigurationType);

            motorPriorities.add(new MotorPriority(motors.get(i),3,5));
        }

        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);

        localizer = new TwoWheelLocalizer(hardwareMap);
        localizer.setIMU(sensors.imu);
    }

    double distanceBeforeMovingToPointsHeading = 6.0;

    public void update () {
        if(!DRIVETRAIN_ENABLED) {return;}

        localizer.update();
        MyPose2d estimate = localizer.getPoseEstimate();
        MyPose2d signal = currentSplineToFollow.getErrorFromNextPoint(estimate); // signal is null when in teleop only in auto do we have signal

        if (signal != null) {
            double errorDistance = Math.sqrt(Math.pow(signal.x,2) + Math.pow(signal.y,2));
            boolean inDist = errorDistance < distanceBeforeMovingToPointsHeading;
            double headingError = inDist ? signal.heading : Math.atan2(signal.y*4.0,signal.x)+currentSplineToFollow.points.get(0).headingOffset; // pointing at the point
            while (Math.abs(headingError) > Math.toRadians(180)) { // moves angle to be within 180 degrees
                headingError -= Math.signum(headingError) * Math.toRadians(360);
            }
            Log.e("headingError", headingError + " " + inDist);
            double fwd = signal.x; // relative x error
            double turn = 2.5*headingError*TRACK_WIDTH/2.0; // s/r = theta
            double[] motorPowers = {
                    fwd - turn,
                    fwd - turn,
                    fwd + turn,
                    fwd + turn
            };
            double max = Math.abs(motorPowers[0]);
            for (int i = 1; i < motorPowers.length; i ++) { // finds max power
                max = Math.max(max, Math.abs(motorPowers[i]));
            }
            double maxSpeed = Math.min(1.0, errorDistance / currentSplineToFollow.minimumRobotDistanceFromPoint); // we want the speed to slow down as we approach the point
            // slow down on turns
            if (currentSplineToFollow.points.size() >= 2) {
                double changeAngle = 0;
                int maxDist = 12; // start slowing down 12 inches from turn
                double k = 0.35;
                int maxIndex = Math.min(maxDist/2-1,currentSplineToFollow.points.size()-1); // since we have points every two inches if we divide maxDist by 2 we get number of points
                for (int i = 0; i < maxIndex; i ++) { // iterating through the next 6 points and taking a kalman filter of change angles
                    // first get change in angle between last point and point in front of it then multiply by a constant k and add in 65 percent of change angle from previous point
                    changeAngle = Math.abs(currentSplineToFollow.points.get(maxIndex - i).heading - currentSplineToFollow.points.get(maxIndex-i-1).heading) * k + changeAngle * (1.0-k);
                }
                maxSpeed *= 1.0 - Math.min(changeAngle/Math.toRadians(5),0.15); // max we slow down to on a turn is 0.85
            }
            maxSpeed = Math.max(maxSpeed,0.5); // minimum max speed
            for (int i = 0; i < motorPowers.length; i ++) {
                motorPowers[i] /= max; // keeps proportions in tack by getting a percentage
                motorPowers[i] *= maxSpeed; // slow down motors
                motorPowers[i] *= 1.0 - MIN_MOTOR_POWER_TO_OVERCOME_FRICTION; // we do this so that we keep proportions when we add MIN_MOTOR_POWER_TO_OVERCOME_FRICTION in the next line below. If we had just added MIN_MOTOR_POWER_TO_OVERCOME_FRICTION without doing this 0.9 and 1.0 become the same motor power
                motorPowers[i] += MIN_MOTOR_POWER_TO_OVERCOME_FRICTION * Math.signum(motorPowers[i]);
                motorPriorities.get(i).setTargetPower(motorPowers[i]);
            }
        }

        if((breakFollowing)
                && (Math.abs(estimate.getX() - targetPose.getX()) < xThreshold)
                && (Math.abs(estimate.getY() - targetPose.getY()) < yThreshold)
                && (Math.abs(estimate.getHeading() - targetPose.getHeading()) < headingThreshold)) {
            breakFollowing();
            setMotorPowers(0,0,0,0);
        }
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public void setMotorPowers(double lf, double lr, double rr, double rf) {
        motorPriorities.get(0).setTargetPower(lf);
        motorPriorities.get(1).setTargetPower(lr);
        motorPriorities.get(2).setTargetPower(rr);
        motorPriorities.get(3).setTargetPower(rf);
    }

    public void drive (Gamepad gamepad) {
        double forward = -0.4*Math.tan(((gamepad.left_stick_y * -1 ) / 0.85));
        double turn = -gamepad.right_stick_x;

        double p1 = forward+turn;
        double p2 = forward+turn;
        double p3 = forward-turn;
        double p4 = forward-turn;
        setMotorPowers(p1, p2, p3, p4);
    }

    boolean breakFollowing = false;
    MyPose2d targetPose = new MyPose2d(0,0,0);
    double xThreshold = 0.5;
    double yThreshold = 0.5;
    double headingThreshold = Math.toRadians(5.0);

    public void setBreakFollowingThresholds (MyPose2d thresholds, MyPose2d targetPose) {
        this.targetPose = targetPose;
        breakFollowing = true;
        xThreshold = thresholds.getX();
        yThreshold = thresholds.getY();
        headingThreshold = thresholds.getHeading();
    }

    public void breakFollowing() {
        currentSplineToFollow.points.clear();
    }

    public MyPose2d getPoseEstimate() {
        return localizer.getPoseEstimate();
    }

    public void setSpline(Spline spline) {
        currentSplineToFollow = spline;
    }

    public void setPoseEstimate(MyPose2d pose2d) {
        localizer.setPoseEstimate(pose2d);
    }

    public boolean isBusy() {
        return currentSplineToFollow.points.size() != 0;
    }

    public void followSplineWithTimer(Spline trajectory, LinearOpMode opMode, long startTime) {
        setSpline(trajectory);
        while(isBusy() && (opMode.opModeIsActive() || (System.currentTimeMillis() - startTime >= 29500 && System.currentTimeMillis() - startTime <= 30800))) {
            update();
        }
    }

    public void followSpline(Spline spline, LinearOpMode opMode) {
        setSpline(spline);
        while(isBusy() && opMode.opModeIsActive()) {
            update();
        }
    }
}
