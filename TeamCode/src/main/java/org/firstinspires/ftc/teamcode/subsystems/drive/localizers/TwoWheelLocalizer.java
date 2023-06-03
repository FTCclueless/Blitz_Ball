package org.firstinspires.ftc.teamcode.subsystems.drive.localizers;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.utils.Encoder;
import org.firstinspires.ftc.teamcode.utils.MyPose2d;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;

import java.util.ArrayList;

public class TwoWheelLocalizer {
    public Encoder[] encoders;
    long lastTime = System.nanoTime();
    public double x = 0;
    public double y = 0;
    public double heading = 0;

    MyPose2d currentPose = new MyPose2d(0,0,0);
    MyPose2d currentVel = new MyPose2d(0,0,0);
    public MyPose2d relCurrentVel = new MyPose2d(0,0,0);
    MyPose2d currentPowerVector = new MyPose2d(0,0,0);

    ArrayList<MyPose2d> poseHistory = new ArrayList<MyPose2d>();
    ArrayList<MyPose2d> relHistory = new ArrayList<MyPose2d>();
    ArrayList<Double> loopTimeHistory = new ArrayList<Double>();

    IMU imu;

    public TwoWheelLocalizer(HardwareMap hardwareMap) {
        encoders = new Encoder[2];

        encoders[0] = new Encoder(new MyPose2d(0,5.5926264886),  -1); // left
        encoders[1] = new Encoder(new MyPose2d(0,-5.6839121499),-1); // right
    }

    public void setIMU(IMU imu){
        this.imu = imu;
    }

    public void updateEncoders(int[] encoders) {
        for (int i = 0; i < this.encoders.length; i ++) {
            this.encoders[i].update(encoders[i]);
        }
    }

    public void setPose(double x, double y, double h){
        this.x = x;
        this.y = y;
        this.heading += h - this.heading;
    }

    public MyPose2d getPoseEstimate() {
        return new MyPose2d(currentPose.x, currentPose.y, currentPose.heading);
    }

    public void setPoseEstimate(MyPose2d pose2d) {
        setPose(pose2d.getX(), pose2d.getY(), pose2d.getHeading());
    }

    public MyPose2d getPoseVelocity() {
        return new MyPose2d(currentVel.x, currentVel.y, currentVel.heading);
    }

    public void update() {
        long currentTime = System.nanoTime();
        double loopTime = (currentTime-lastTime)/1.0e9;
        lastTime = currentTime;

        double deltaLeft = encoders[0].getDelta();
        double deltaRight = encoders[1].getDelta();
        double leftY = encoders[0].y;
        double rightY = encoders[1].y;

        // This is the heading because the heading is proportional to the difference between the left and right wheel.
        double deltaHeading = (deltaRight - deltaLeft)/(leftY-rightY);
        // This is a weighted average for the amount moved forward with the weights being how far away the other one is from the center
        double relDelta = (deltaRight*leftY - deltaLeft*rightY)/(leftY-rightY);

        relHistory.add(0,new MyPose2d(relDelta,0.0,deltaHeading));

        if (deltaHeading != 0) { // this avoids the issue where deltaHeading = 0 and then it goes to undefined. This effectively does L'Hopital's
            double r1 = relDelta / deltaHeading;
            relDelta = Math.sin(deltaHeading) * r1 - (1.0 - Math.cos(deltaHeading));
        }
        x += relDelta * Math.cos(heading);
        y += relDelta * Math.sin(heading);

        heading += deltaHeading;

        currentPose = new MyPose2d(x, y, heading);

        loopTimeHistory.add(0,loopTime);
        poseHistory.add(0,currentPose);
        updateVelocity();
        updateTelemetry();
    }

    public void updateTelemetry() {
        TelemetryUtil.packet.put("x", currentPose.getX());
        TelemetryUtil.packet.put("y", currentPose.getY());
        TelemetryUtil.packet.put("heading (deg)", Math.toDegrees(currentPose.getHeading()));
    }

    // powers = [leftFront, leftBack, rightFront, rightBack]
    public void updatePowerVector(double[] powers) {
        for (int i = 0; i < powers.length; i ++){
            powers[i] = Math.max(Math.min(powers[i],1),-1);
        }
        double forward = (powers[0] + powers[1] + powers[2] + powers[3]) / 4;
        double left = (-powers[0] + powers[1] - powers[2] + powers[3]) / 4; // left power is less than 1 of forward power
        double turn = (-powers[0] - powers[1] + powers[2] + powers[3]) / 4;
        currentPowerVector.x = forward * Math.cos(heading) - left * Math.sin(heading);
        currentPowerVector.y = left * Math.cos(heading) + forward * Math.sin(heading);
        currentPowerVector.heading = turn;
    }

    public void updateVelocity() {
        double targetVelTimeEstimate = 0.2; // in seconds
        double actualVelTime = 0;
        double relDeltaXTotal = 0;
        double relDeltaYTotal = 0;
        double totalTime = 0;
        int lastIndex = 0;

        // looks through past loop times until the last loop time that is under the targetVelTimeEstimate
        for (int i = 0; i < loopTimeHistory.size(); i++){
            totalTime += loopTimeHistory.get(i);
            if (totalTime <= targetVelTimeEstimate) {
                actualVelTime += loopTimeHistory.get(i);
                relDeltaXTotal += relHistory.get(i).getX();
                relDeltaYTotal += relHistory.get(i).getY();
                lastIndex = i;
            } else {
                break;
            }
        }

        double averageHeadingVel = (poseHistory.get(0).getHeading() - poseHistory.get(lastIndex).getHeading()) / actualVelTime;

        // global velocity
        currentVel = new MyPose2d(
                (poseHistory.get(0).getX() - poseHistory.get(lastIndex).getX()) / actualVelTime,
                (poseHistory.get(0).getY() - poseHistory.get(lastIndex).getY()) / actualVelTime,
                averageHeadingVel
        );
        // relative velocity (can't do final minus initial because relative has a heading component)
        relCurrentVel = new MyPose2d(
                (relDeltaXTotal) / actualVelTime,
                (relDeltaYTotal) / actualVelTime,
                averageHeadingVel
        );

        // clearing arrays
        while (lastIndex + 1 < loopTimeHistory.size()) {
            loopTimeHistory.remove(loopTimeHistory.size() - 1);
            relHistory.remove(relHistory.size() - 1);
            poseHistory.remove(poseHistory.size() - 1);
        }
    }
}
