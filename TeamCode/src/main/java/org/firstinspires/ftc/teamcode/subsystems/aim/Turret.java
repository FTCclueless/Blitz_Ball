package org.firstinspires.ftc.teamcode.subsystems.aim;



import android.util.Log;

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

import java.util.ArrayList;

@Config
public class Turret {

    private Sensors sensors;


    public final double ticksPerRadian = 111.261143192; //145.090909091 / 0.20754716981 (22/106) / 2pi

    public static final double maxRotation = Math.toRadians(180);
    public static double maxAccel = 8.98786378884; // 1000 / 111.261143192
    public TurretState turretState = TurretState.MANUAL_CONTROL;
    public static boolean pidEnabled = false;
    public static PID turretPid = new PID(0.1,0,0);


    public double targetAngle = 0;
    public double currentAngle = 0;

    DcMotorEx turretMotor;
    public double errorAngle;

    public double turretVelocity;
    double turretPower;
    ArrayList<MotorPriority> motorPriorities;

    public static double AngleToSlowDown = 2; //in radians
    public double powPerVel = 0.04257493408921866; //velocity per power
    public double minPowToOvercomeFriction = 0.07498230133005577; //minimum power to overcome friction
    public double maxVelocity = (1.0-minPowToOvercomeFriction)/ powPerVel; //21.797296059415; // 2540 / 111.261143192
    public static double accelMult = 0.13;
    public static double errorMargin = Math.toRadians(3);

    public double offsetVel = 0;



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

    public void setTargetAngle(double angle){
        setTargetAngle(angle,0);
    }

    public void setTargetAngle(double angle, double vel) {
        this.targetAngle = angle;
        this.offsetVel = vel;
        while (Math.abs(targetAngle) > Math.PI) {
            targetAngle -= Math.PI * 2.0 * Math.signum(targetAngle);
        }
        targetAngle = Math.min(Math.max(targetAngle,-1.0 * maxRotation),maxRotation);
        errorAngle = targetAngle - currentAngle;
        while (Math.abs(errorAngle) > Math.PI) {
            errorAngle -= Math.PI * 2.0 * Math.signum(errorAngle);
        }

        if (Math.abs(currentAngle+errorAngle) > maxRotation) { //would I go out of bounds if I follow?
            errorAngle -= Math.signum(errorAngle) * 2.0 * Math.PI; // add 360 to stay in bounds
            /* no longer needed due to clips on targetAngle
            if (maxRotation < Math.PI) { //check if you can reach everything
                if (Math.abs(currentAngle+errorAngle) > maxRotation){ //check if new way around still puts us out
                    Log.e("Turret", "Angle out of range");
                    //find closer angle between maxRotation and -maxRotation from targetAngle
                    targetAngle = maxRotation * Math.signum(targetAngle);
                    errorAngle = targetAngle - currentAngle;
                }
            }
            */
        }

    }

    public boolean isComplete(double errorMargin) {
        return Math.abs(targetAngle - currentAngle) <= errorMargin;
    }

    public void move(Gamepad gamepad) {
        motorPriorities.get(4).setTargetPower(gamepad.right_stick_x);
    }

    private double feedForward() {
        double targetSpeed = errorAngle * maxVelocity/ AngleToSlowDown;
        targetSpeed = Math.max(Math.min(targetSpeed, maxVelocity), -maxVelocity);
        targetSpeed -= offsetVel;
        double targetPower = targetSpeed * powPerVel + (targetSpeed - turretVelocity)/maxVelocity * accelMult + minPowToOvercomeFriction * ((Math.abs(errorAngle) > errorMargin) ? Math.signum(targetSpeed): 0);
        targetPower = Math.min(Math.max(targetPower,-1),1);
        return targetPower;
    }



    public void update() {
        currentAngle = -1 * sensors.getTurretAngle() / ticksPerRadian;
        errorAngle = targetAngle - currentAngle;
        turretVelocity = sensors.getTurretVelocity() / ticksPerRadian;

        switch (turretState) {
            case MANUAL_CONTROL:
                // FIXME: ask later if we should pass gamepad to all updates
                break;
            case AUTOAIM:

                turretPower = feedForward();
                TelemetryUtil.packet.put("turretPow", turretPower);

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
