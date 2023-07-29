package org.firstinspires.ftc.teamcode.subsystems.drive.aim;



import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.MotorPriority;
import org.firstinspires.ftc.teamcode.utils.PID;
import org.firstinspires.ftc.teamcode.utils.TrapezoidMotionProfile;

import java.util.ArrayList;

@Config
public class Turret {
    private Sensors sensors;

    public static final double maxRotation = Math.toRadians(270);
    public static double maxVelocity = 2540;
    public static double maxAccel = 2000;
    public TurretState turretState = TurretState.MANUAL_CONTROL;
    public static boolean pidEnabled = false;
    public static PID turretPid = new PID(1,0,0);

    private final double ticksPerRadian = 30.113207547; //145.090909091 * 0.20754716981 (22/106)



    public double targetAngle;
    public double currentAngle;
    public double zeroAngle;

    DcMotorEx turretMotor;
    public double errorAngle;

    double turretVelocity;
    double turretPower;
    ArrayList<MotorPriority> motorPriorities;

    public static TrapezoidMotionProfile turretMotionProfile = new TrapezoidMotionProfile(maxAccel,maxVelocity);
    public static double errorMargin;



    public Turret(HardwareMap hardwareMap, ArrayList<MotorPriority> motorPriorities, Sensors sensors) {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.sensors = sensors;


        zeroAngle = sensors.getTurretAngle() / ticksPerRadian;
        targetAngle = currentAngle;
        turretVelocity = sensors.getTurretVelocity() / ticksPerRadian;

        this.motorPriorities = motorPriorities;
        motorPriorities.add(new MotorPriority(turretMotor, 3, 5));



    }

    public void setTargetAngle(double angle) {
        while (Math.abs(angle) > maxRotation) {
            angle -= Math.PI * 2.0 * Math.signum(angle);
        }
        turretMotionProfile.setTargetPos(angle, currentAngle, turretVelocity);

        this.targetAngle = angle;

    }

    public boolean isComplete(double errorMargin) {
        return errorAngle - currentAngle <= errorMargin;
    }

    public void move(Gamepad gamepad) {
        motorPriorities.get(4).setTargetPower(gamepad.right_stick_x);
    }



    public void update() {
        currentAngle = sensors.getTurretAngle() / ticksPerRadian - zeroAngle;
        errorAngle = targetAngle - currentAngle;
        turretVelocity = sensors.getTurretVelocity() / ticksPerRadian;

        switch (turretState) {
            case MANUAL_CONTROL:
                // FIXME: ask later if we should pass gamepad to all updates
                break;
            case AUTOAIM:
                turretPower = turretMotionProfile.getTargetVel(currentAngle);
                motorPriorities.get(4).setTargetPower(turretPower);
                /*if (isComplete(errorMargin)) {
                    turretState = TurretState.AUTOAIM;
                }*/
                break;
            case PID_ENABLED:
                turretPower = turretPid.getOut(errorAngle);


                motorPriorities.get(4).setTargetPower(turretPower);
                break;
        }
    }
}
