package org.firstinspires.ftc.teamcode.opmodes.testing.valueYoinkin;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.aim.Turret;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;

@TeleOp
public class TurretValueYoinker extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        double maxVel = 0;
        double maxAccel = 0;
        double time = 0;
        double lastTime = 0;
        double vel = 0;
        double lastVel = 0;
        double counter = 0;
        double sumPow = 0;
        double sumPow2 = 0;
        double sumVel = 0;
        double sumVel2 = 0;
        double sumProduct = 0;


        waitForStart();

        Robot robot = new Robot(hardwareMap);
        Turret turret = robot.turret;





        while (!isStopRequested() && counter < 10000) {
            counter += 1;
            robot.update();
            time = System.currentTimeMillis();
            vel = ((double)robot.sensors.getTurretVelocity()) / 111.261143192;
            double power = counter/10000.0;
            sumPow += power;
            sumPow2 += power*power;
            sumVel += vel;
            sumVel2 += vel*vel;
            sumProduct += (vel*power);

            ((PriorityMotor) robot.hardwareQueue.getDevice("turret")).setTargetPower(power);
            robot.update();




            // System.out.println("***** accel: " + accel + " " + "avg: " + sum/counter + " time: " + (time-lastTime));

            lastTime = time;
            lastVel = vel;



            }
        double yIntercept = (sumPow*sumVel2 - sumVel*sumProduct) / (counter * sumVel2 - Math.pow(sumVel, 2));
        double slope = (counter * sumProduct - sumVel*sumPow) / (counter*sumVel2 - Math.pow(sumVel,2));

        Log.e("yIntercept", "" + yIntercept);
        Log.e("slope", "" + slope);




        }
    }

