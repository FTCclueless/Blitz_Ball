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
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.TrapezoidMotionProfile;

import java.util.ArrayList;

@Config
public class Turret {
    public static double minPower = 0.08;


    private Sensors sensors;


    public final double ticksPerRadian = 30.113207547; //145.090909091 * 0.20754716981 (22/106)

    public static final double maxRotation = Math.toRadians(270);
    public static double maxVelocity = 84.32837105944; // 2540 / 30.113207547
    public static double maxAccel = 33.20802005031258; // 1000 / 30.113207547
    public TurretState turretState = TurretState.MANUAL_CONTROL;
    public static boolean pidEnabled = false;
    public static PID turretPid = new PID(1,0,0);





    public double targetAngle;
    public double currentAngle = 0;

    DcMotorEx turretMotor;
    public double errorAngle;

    double turretVelocity;
    double turretPower;
    ArrayList<MotorPriority> motorPriorities;

    public static TrapezoidMotionProfile turretMotionProfile = new TrapezoidMotionProfile(maxAccel,maxVelocity, 1);
    public static double errorMargin;
    public static double target;



    public Turret(HardwareMap hardwareMap, ArrayList<MotorPriority> motorPriorities, Sensors sensors) {


        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.sensors = sensors;

        targetAngle = currentAngle;
        turretVelocity = 0;

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
        currentAngle = sensors.getTurretAngle() / ticksPerRadian;
        TelemetryUtil.packet.put("sensorTargetAngle", sensors.getTurretAngle() / ticksPerRadian);
        errorAngle = targetAngle - currentAngle;
        turretVelocity = sensors.getTurretVelocity() / ticksPerRadian;

        switch (turretState) {
            case MANUAL_CONTROL:
                // FIXME: ask later if we should pass gamepad to all updates
                break;
            case AUTOAIM:
                turretPower = turretMotionProfile.getTargetVel(currentAngle)/ maxVelocity;
                if (Math.abs(turretPower) < minPower) {
                    turretPower = minPower * Math.signum(turretPower);
                }

                motorPriorities.get(4).setTargetPower(turretPower);
                TelemetryUtil.packet.put("***************** power ", turretPower);
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
