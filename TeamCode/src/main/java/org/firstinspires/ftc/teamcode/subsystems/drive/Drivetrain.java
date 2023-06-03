package org.firstinspires.ftc.teamcode.subsystems.drive;

import static org.firstinspires.ftc.teamcode.utils.Globals.DRIVETRAIN_ENABLED;
import static org.firstinspires.ftc.teamcode.utils.Globals.MAX_DRIVETRAIN_SPEED;
import static org.firstinspires.ftc.teamcode.utils.Globals.MIN_MOTOR_POWER_TO_OVERCOME_FRICTION;
import static org.firstinspires.ftc.teamcode.utils.Globals.TRACK_WIDTH;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    private Sensors sensors;

    public TwoWheelLocalizer localizer;

    public Spline currentSplineToFollow = new Spline(new MyPose2d(0,0,0));

    public Drivetrain(HardwareMap hardwareMap, ArrayList<MotorPriority> motorPriorities, Sensors sensors) {
        this.motorPriorities = motorPriorities;
        this.sensors = sensors;

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

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);

        localizer = new TwoWheelLocalizer(hardwareMap);
        localizer.setIMU(sensors.getImu());
    }

    public void update () {
        if(!DRIVETRAIN_ENABLED) {return;}

        updateLocalizer();

        MyPose2d estimate = localizer.getPoseEstimate();
        MyPose2d signal = currentSplineToFollow.getErrorFromNextPoint(estimate); // signal is null when in teleop only in auto do we have signal

        if (signal != null) {
            double errorDistance = Math.sqrt(Math.pow(signal.x,2) + Math.pow(signal.y,2));
            boolean inDist = (currentSplineToFollow.points.get(0).mustGoToPoint || currentSplineToFollow.points.size() == 1) && errorDistance < 6.0;
            double headingError = inDist ? signal.heading : Math.atan2(signal.y,signal.x);// pointing at the point
            headingError += currentSplineToFollow.points.get(0).headingOffset;
            while (Math.abs(headingError) > Math.toRadians(180)) { // moves angle to be within 180 degrees
                headingError -= Math.signum(headingError) * Math.toRadians(360);
            }
            double maxRadius = MyPose2d.maxDistanceFromPoint;
            double minRadius = MyPose2d.minDistanceFromPoint;
            double radius = currentSplineToFollow.points.get(0).radius;
            for (int i = 1; i < Math.min(currentSplineToFollow.points.size()-1,5); i ++){
                radius = Math.min(currentSplineToFollow.points.get(i).radius,radius);
            }
            double maxSpeedPercentage = Math.max(Math.min((radius-minRadius)/(maxRadius-minRadius),0.5),0.25); // we want the speed to slow down as we approach the point & minimum max speed

            double currentFwdPercentage = Math.min(Math.abs(localizer.relCurrentVel.x/MAX_DRIVETRAIN_SPEED),1.0);

            double fwd = signal.x * (maxSpeedPercentage*3.5 - currentFwdPercentage); // relative x error
            double turn = headingError*TRACK_WIDTH/4; // s/r = theta
            double[] motorPowers = {
                    fwd - turn,
                    fwd - turn,
                    fwd + turn,
                    fwd + turn
            };
            double max = 1.0;
            for (int i = 0; i < motorPowers.length; i ++) { // finds max power
                max = Math.max(max, Math.abs(motorPowers[i]));
            }
            for (int i = 0; i < motorPowers.length; i ++) {
                motorPowers[i] /= max; // keeps proportions in tack by getting a percentage
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

    public void updateLocalizer() {
        localizer.updateEncoders(sensors.getOdometry());
        localizer.update();
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
        double forward = 0.45*Math.tan(((gamepad.left_stick_y * -1 ) / 0.85));
        double turn = gamepad.right_stick_x;

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
}
