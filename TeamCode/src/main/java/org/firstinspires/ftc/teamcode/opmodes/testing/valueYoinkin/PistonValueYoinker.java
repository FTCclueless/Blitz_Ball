package org.firstinspires.ftc.teamcode.opmodes.testing.valueYoinkin;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.aim.Aim;
import org.firstinspires.ftc.teamcode.subsystems.aim.Turret;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;

@Config
@TeleOp
public class PistonValueYoinker extends LinearOpMode {
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
        robot.aim.setState(Aim.State.OFF);
        Transfer transfer = robot.aim.transfer;





        while (!isStopRequested() && counter < 10000) {
            counter += 1;
            robot.update();
            time = System.currentTimeMillis();
            vel = ((double)robot.sensors.getPistonVelocity()) / 85.5759958182;
            double power = counter/10000.0;
            sumPow += power;
            sumPow2 += power*power;
            sumVel += vel;
            sumVel2 += vel*vel;
            sumProduct += (vel*power);
            System.out.println(vel);

            ((PriorityMotor) robot.hardwareQueue.getDevice("piston")).setTargetPower(power);
            robot.update();




            // System.out.println("***** accel: " + accel + " " + "avg: " + sum/counter + " time: " + (time-lastTime));

            lastTime = time;
            lastVel = vel;



        }
        System.out.printf("%f %f\n", counter, sumVel2);
        double yIntercept = (sumPow*sumVel2 - sumVel*sumProduct) / (counter * sumVel2 - Math.pow(sumVel, 2));
        double slope = (counter * sumProduct - sumVel*sumPow) / (counter*sumVel2 - Math.pow(sumVel,2));

        Log.e("yIntercept", "" + yIntercept);
        Log.e("slope", "" + slope);





    }
}
