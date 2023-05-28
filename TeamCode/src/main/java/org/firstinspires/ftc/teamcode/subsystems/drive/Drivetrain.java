package org.firstinspires.ftc.teamcode.subsystems.drive;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.subsystems.drive.localizers.TwoWheelLocalizer;
import org.firstinspires.ftc.teamcode.utils.MotorPriority;
import org.firstinspires.ftc.teamcode.utils.MyPose2d;

import java.util.ArrayList;
import java.util.List;

public class Drivetrain {
    public DcMotorEx leftFront, leftRear, rightRear, rightFront;
    public DcMotorEx[] motors;

    private BNO055IMU imu;
    private VoltageSensor batteryVoltageSensor;

    private ArrayList<MotorPriority> motorPriorities;

    public TwoWheelLocalizer localizer;

    public Spline currentSplineToFollow = new Spline(new MyPose2d(0,0,0));

    public Drivetrain(HardwareMap hardwareMap, ArrayList<MotorPriority> motorPriorities) {
        this.motorPriorities = motorPriorities;

        motors = new DcMotorEx[]{
            hardwareMap.get(DcMotorEx.class, "leftFront"),
            hardwareMap.get(DcMotorEx.class, "leftRear"),
            hardwareMap.get(DcMotorEx.class, "rightRear"),
            hardwareMap.get(DcMotorEx.class, "rightFront")
        };

        for (DcMotorEx m : motors) {
            motorPriorities.add(new MotorPriority(m, 3, 5));
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setDirection(DcMotor.Direction.REVERSE);
        }
    }
}
