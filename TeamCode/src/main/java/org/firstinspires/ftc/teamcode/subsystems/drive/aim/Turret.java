package org.firstinspires.ftc.teamcode.subsystems.drive.aim;

import static org.firstinspires.ftc.teamcode.utils.Globals.TICKS_PER_RADIAN;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.AngleUtil;
import org.firstinspires.ftc.teamcode.utils.MotorPriority;
import org.firstinspires.ftc.teamcode.utils.PID;
import org.firstinspires.ftc.teamcode.utils.TrapezoidMotionProfile;

import java.util.ArrayList;

enum State {
    PID_ENABLED,
    AUTOAIM,
    MANUAL_CONTROL
}

@Config
public class Turret {
    public static final double maxRotation = Math.toRadians(270);
    public static double maxVelocity = 2540;
    public static double maxAccel = 2000;
    public State state = State.MANUAL_CONTROL;
    public static boolean pidEnabled = false;
    public static PID turretPid = new PID(1,0,0);


    final double turretGearRatio = 0.20754716981; // 22/106


    public double targetAngle;
    public double currentAngle;
    DcMotorEx turretMotor;
    double errorAngle;

    double turretVelocity;
    double turretPower;
    ArrayList<MotorPriority> motorPriorities;

    public static TrapezoidMotionProfile turretMotionProfile = new TrapezoidMotionProfile(maxAccel,maxVelocity);



    public Turret(HardwareMap hardwareMap, ArrayList<MotorPriority> motorPriorities) {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        currentAngle = turretMotor.getCurrentPosition() / TICKS_PER_RADIAN / turretGearRatio;
        targetAngle = currentAngle;

        this.motorPriorities = motorPriorities;
        motorPriorities.add(new MotorPriority(turretMotor, 3, 5));



    }

    public void setTargetAngle(double angle) {
        while (Math.abs(angle) > maxRotation) {
            angle -= Math.PI * 2.0 * Math.signum(angle);
        }

        this.targetAngle = angle;

    }

    public boolean isComplete(double errorMargin) {
        return errorAngle - currentAngle <= errorMargin;
    }

    public void move(Gamepad gamepad) {
        motorPriorities.get(4).setTargetPower(gamepad.right_stick_x);
    }



    public void update() {
        switch (state) {
            case MANUAL_CONTROL:
                // FIXME: ask later if we should pass gamepad to all updates
                break;
            case AUTOAIM:
                break;
            case PID_ENABLED:
                currentAngle = turretMotor.getCurrentPosition() / TICKS_PER_RADIAN / turretGearRatio;
                errorAngle = AngleUtil.clipAngle(targetAngle - currentAngle);
                turretPower = turretPid.getOut(errorAngle);

                //do motion profiling

                motorPriorities.get(4).setTargetPower(turretPid.getOut(errorAngle));
                break;
        }
    }
}
