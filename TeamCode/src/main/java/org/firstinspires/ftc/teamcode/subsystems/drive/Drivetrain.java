package org.firstinspires.ftc.teamcode.subsystems.drive;

import static org.firstinspires.ftc.teamcode.utils.Globals.DRIVETRAIN_ENABLED;
import static org.firstinspires.ftc.teamcode.utils.Globals.MIN_MOTOR_POWER_TO_OVERCOME_FRICTION;
import static org.firstinspires.ftc.teamcode.utils.Globals.TRACK_WIDTH;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.subsystems.drive.localizers.TwoWheelLocalizer;
import org.firstinspires.ftc.teamcode.utils.MotorPriority;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.Vector2;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Config
public class Drivetrain {
    // Pure pursuit tuning values
    public static double lookAheadRadius = 13;
    public static double maxDeviationFromPath = 12;
    public static double speed = 0.7;
    public static double curvyCompVariable = 20;
    public static int futureIndexes = 6; //each index is inchesPerPointGenerated apart (pre-calculated radius)
    public static int pastIndexes = 15; //each index is 1 loop apart (instantaneous dynamic radius)
    public static double futurePastWeight = 0.6; //weight of future to past
    public static double maxRadiusSum = 35; //needed so outliers don't completely wreck havoc on average



    public DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private List<DcMotorEx> motors;

    private ArrayList<MotorPriority> motorPriorities;
    private Sensors sensors;

    public TwoWheelLocalizer localizer;
    private boolean doNotMove = false;

    private Spline currentPath = null;
    private int pathIndex = 0;

    private double[] prevR = new double[pastIndexes];
    int prevRIndex = 0;
    boolean pastRFull = false;
    double sumPrev = 0;

    public Drivetrain(HardwareMap hardwareMap, ArrayList<MotorPriority> motorPriorities, Sensors sensors) {
        this.motorPriorities = motorPriorities;
        this.sensors = sensors;

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (int i = 0; i < motors.size(); i++) {
            MotorConfigurationType motorConfigurationType = motors.get(i).getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motors.get(i).setMotorType(motorConfigurationType);

            motorPriorities.add(new MotorPriority(motors.get(i), 3, 5));
        }

        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);

        localizer = new TwoWheelLocalizer(hardwareMap);
        localizer.setIMU(sensors.getImu());
    }

    public void setCurrentPath(Spline path) {
        currentPath = path;
        pathIndex = 0;
        doNotMove = false;
    }

    public Spline getCurrentPath() {
        return currentPath;
    }

    double maxHeadingError = Math.toRadians(95);

    public void update() {
        if (!DRIVETRAIN_ENABLED) {
            return;
        }

        updateLocalizer();

        Canvas canvas = TelemetryUtil.packet.fieldOverlay();
        Pose2d estimate = localizer.getPoseEstimate();

        TelemetryUtil.packet.put("done", doNotMove);
        if (doNotMove) {
            // TODO its kinda bad
            for (DcMotorEx motor : motors) {
                motor.setPower(0);
            }
            return;
        }

        if (currentPath != null) {
            /*while (estimate.getDistanceFromPoint(currentPath.poses.get(pathIndex)) <= 8) {
                if (pathIndex == currentPath.poses.size() - 1) {
                    doNotMove = true;
                    return;
                }

                pathIndex++;
            }*/

            double distance = currentPath.poses.get(pathIndex).getDistanceFromPoint(estimate);
            for (int i = pathIndex + 1; i < currentPath.poses.size(); i++) {
                double tdistance = currentPath.poses.get(i).getDistanceFromPoint(estimate);
                if (tdistance < distance) {
                    distance = tdistance;
                    pathIndex = i;
                }
            }

            if (currentPath.poses.get(pathIndex).reversed) {
                estimate.heading += Math.PI;
            }

            if (pathIndex >= currentPath.poses.size() - 1) {
                // Do this well later
                doNotMove = true;
                return;
            }

            TelemetryUtil.packet.put("pathIndex", pathIndex + "/" + currentPath.poses.size());

            Vector2 temp;
            double tempLookAheadR = Drivetrain.lookAheadRadius;
            Vector2 lookAhead = null;

            while (lookAhead == null) {
                for (int i = pathIndex; i < currentPath.poses.size() - 1; i++) {
                    if (pathIndex != currentPath.poses.size()) {

                        temp = lineCircleIntersection(currentPath.poses.get(i), currentPath.poses.get(i + 1), estimate, tempLookAheadR);
                        if (temp != null) {
                            pathIndex = i;
                            lookAhead = temp;
                        }
                    }
                }
                tempLookAheadR += 0.05;
                if (tempLookAheadR >= maxDeviationFromPath) {
                    Pose2d temptemp = null;
                    if (pathIndex < currentPath.poses.size() - 1) {
                        temptemp = currentPath.poses.get(pathIndex + 1);
                    } else {
                        temptemp = currentPath.poses.get(pathIndex);
                    }
                    lookAhead = new Vector2(temptemp.x, temptemp.y);
                    break;
                }
            }

            // Plot the lookahead point
            canvas.setFill("#ff0000");
            canvas.fillCircle(lookAhead.x, lookAhead.y, 1.5);

            Pose2d error = new Pose2d(
                lookAhead.x - estimate.x,
                lookAhead.y - estimate.y,
                0
            );

            /*double a = -Math.tan(AngleUtil.clipAngle(estimate.heading));
            double b = 1;
            double c = Math.tan(AngleUtil.clipAngle(estimate.heading))*estimate.x-estimate.y;
            TelemetryUtil.packet.put("abc", a + " " + b + " " + c);*/

            double relativeErrorY = error.y * Math.cos(estimate.heading) - error.x * Math.sin(estimate.heading);

            double relativeErrorX = Math.abs(Math.sqrt(Math.abs(Math.sqrt(error.x * error.x + error.y * error.y) - Math.pow(relativeErrorY, 2))));
            TelemetryUtil.packet.put("rel_error", relativeErrorX + " " + relativeErrorY);

            double radius = (error.x * error.x + error.y * error.y) / (2 * relativeErrorY);
            double theta = Math.atan2(relativeErrorY, relativeErrorX);

            // Plot the circle thing
            Vector2 perp = new Vector2(-Math.sin(estimate.heading), Math.cos(estimate.heading));
            if (Math.abs(radius) < 25) { // Don't put radius if it will explode ftc dashboard
                canvas.setStroke("#0000ff");
                perp.norm();
                perp.mul(radius);
                perp.mul(Math.signum(radius));
                perp.add(new Vector2(estimate.x, estimate.y));
                canvas.strokeLine(estimate.x, estimate.y, perp.x, perp.y);
                canvas.strokeCircle(perp.x, perp.y, Math.abs(radius));
            }

            /*canvas.setStroke("#00ffff");
            canvas.setStroke("#0000ff");
            perp = new Vector2(-Math.sin(estimate.heading), Math.cos(estimate.heading));
            perp.norm();
            perp.mul(relativeErrorY);
            perp.add(new Vector2(estimate.x, estimate.y));
            canvas.strokeLine(estimate.x, estimate.y, perp.x, perp.y);

            TelemetryUtil.packet.put("radius", radius);
            TelemetryUtil.packet.put("theta", theta);*/



            // Clippy clippy
            int start = pathIndex;
            int end = pathIndex + futureIndexes;
            if (start < 0) {
                start = 0;
            }
            if (end > currentPath.poses.size()) {
                end = currentPath.poses.size();
            }

            double averageFutureR = 0;
            for (int i = start; i < end; i++) {
                averageFutureR += Math.abs(currentPath.poses.get(i).radius);
            }
            if (averageFutureR == 0) {
                // No breaking allowed
                averageFutureR = Math.abs(radius);
            } else {
                averageFutureR /= end - start;
            }
            TelemetryUtil.packet.put("I HATE THIS", averageFutureR);




            sumPrev += Math.min(Math.abs(radius), maxRadiusSum);

            if (pastRFull) {
                sumPrev -= prevR[prevRIndex];
            }
            prevR[prevRIndex] = Math.min(Math.abs(radius),maxRadiusSum);

            prevRIndex++;
            if (prevRIndex >= pastIndexes) {
                prevRIndex = 0;
                pastRFull = true;
            }

            double averagePrevR;
            if (pastRFull) {
                averagePrevR = sumPrev/pastIndexes;
            }
            else {
                averagePrevR = sumPrev/prevRIndex;
            }

            double weighedR = futurePastWeight*averageFutureR + (1.0-futurePastWeight)*averagePrevR;




            TelemetryUtil.packet.put("Reversed", currentPath.poses.get(pathIndex).reversed);
            double turn = TRACK_WIDTH / 2 / radius;
            double fwd = currentPath.poses.get(pathIndex).reversed ? -1 : 1;
            double[] motorPowers = {
                    fwd - turn,
                    fwd - turn,
                    fwd + turn,
                    fwd + turn
            };
            TelemetryUtil.packet.put("fwd", fwd);
            TelemetryUtil.packet.put("turn", turn);
            TelemetryUtil.packet.put("radius", radius);
            TelemetryUtil.packet.put("tempLookR", tempLookAheadR);

            // Post 1 normalization
            double max = 1;
            for (double power : motorPowers) {
                max = Math.max(max, power);
            }


            TelemetryUtil.packet.put("average futurePast", weighedR);
            TelemetryUtil.packet.put("instantaneous R", currentPath.poses.get(pathIndex).radius);

            for (int i = 0; i < motorPowers.length; i++) {
                motorPowers[i] /= max;
                motorPowers[i] *= Math.min(Math.abs(weighedR)/curvyCompVariable, 1); // THIS WORKS AND KYLE DOES NOT KNOW WHY
                motorPowers[i] *= speed;
                motorPowers[i] *= 1.0 - MIN_MOTOR_POWER_TO_OVERCOME_FRICTION; // we do this so that we keep proportions when we add MIN_MOTOR_POWER_TO_OVERCOME_FRICTION in the next line below. If we had just added MIN_MOTOR_POWER_TO_OVERCOME_FRICTION without doing this 0.9 and 1.0 become the same motor power
                motorPowers[i] += MIN_MOTOR_POWER_TO_OVERCOME_FRICTION * Math.signum(motorPowers[i]);
                TelemetryUtil.packet.put("Max", max);
                TelemetryUtil.packet.put("Motor power", motorPowers[0] + " " + motorPowers[1] + " " + motorPowers[2] + " " + motorPowers[3]);

                //motors.get(i).setPower(motorPowers[i]);
                motorPriorities.get(i).setTargetPower(motorPowers[i]);
            }
        }

        /*// pure pursuit follower
        if (error != null) {
            double errorDistance = Math.sqrt(Math.pow(error.x,2) + Math.pow(error.y,2)); // distance equation
            boolean mustGoToPoint = (currentSplineToFollow.points.get(0).mustGoToPoint || currentSplineToFollow.points.size() == 1) && errorDistance < 10.0;
            double headingError = mustGoToPoint ? error.heading : Math.atan2(error.y,error.x) + currentSplineToFollow.points.get(0).headingOffset; // if we want to go to point then we go to the heading otherwise we point to point
            headingError = AngleUtil.clipAngle(headingError);

            double maxRadius = Pose.maxDistanceFromPoint;
            double minRadius = Pose.minDistanceFromPoint;
            double smallestRadiusOfNextPoints = currentSplineToFollow.points.get(0).radius;
            TelemetryUtil.packet.put("radius", smallestRadiusOfNextPoints);
            //for (int i = 1; i < Math.min(currentSplineToFollow.points.size()-1,1); i++)  { // finding smallest radius for next 5 points
            //    smallestRadiusOfNextPoints = Math.min(currentSplineToFollow.points.get(i).radius,smallestRadiusOfNextPoints);
            //}

            double speedFromRadiusPercentage = (smallestRadiusOfNextPoints-minRadius)/(maxRadius-minRadius); // Maximum forward speed based on the upcoming radius
            double speedFromHeadingErrorPercentage = Math.max((maxHeadingError - Math.abs(headingError))/maxHeadingError,0); // Maximum forward speed based on the current heading error
            double speedFromEndPercentage = mustGoToPoint ? Math.abs(error.x) / speedFromEndDiv : 1; // slows down the robot when it reaches an end

            double fwdSpeedPercentage = Math.min(speedFromRadiusPercentage,speedFromHeadingErrorPercentage);
            fwdSpeedPercentage = speedFromEndPercentage * Math.max(Math.min(fwdSpeedPercentage,maxSpeed),minSpeed); // we want the speed to slow down as we approach the point & minimum max speed
            double currentFwdPercentage = Math.min(Math.abs(localizer.relCurrentVel.x/MAX_DRIVETRAIN_SPEED),1.0);
            double currentTurnPercentage = Math.min(Math.abs(localizer.relCurrentVel.heading/TRACK_WIDTH), 1.0);

            double breakingFactor = 0.45; // scale factor for how much you wanna weigh current forward percentage into braking
            double differenceBetweenSetAndActual = fwdSpeedPercentage - currentFwdPercentage;

            double fwd = Math.signum(error.x) * (fwdSpeedPercentage + Math.max(differenceBetweenSetAndActual * breakingFactor, facDICKS)); // applies breaking power to slow it down, most breaking power applied is -0.3
            double turn = TRACK_WIDTH/2*headingError; // s=r*theta
            turn *= turnMultiplier;
            if (Math.abs(headingError) > Math.toRadians(slowBelowDeg) && Math.abs(headingError) < Math.toRadians(slowAboveDeg) && currentTurnPercentage > slowPercentageThresh) {
                turn = -turnSlownessAfterTurn * Math.signum(turn);
            }

            double[] motorPowers = {
                    fwd - turn,
                    fwd - turn,
                    fwd + turn,
                    fwd + turn
            };
            double max = 1.0;
            for (double motorPower : motorPowers) { // finds max power if greater than 1.0
                max = Math.max(max, Math.abs(motorPower));
            }
            for (int i = 0; i < motorPowers.length; i ++) {
                motorPowers[i] /= max; // keeps proportions in tack by getting a percentage
                motorPowers[i] *= 1.0 - MIN_MOTOR_POWER_TO_OVERCOME_FRICTION; // we do this so that we keep proportions when we add MIN_MOTOR_POWER_TO_OVERCOME_FRICTION in the next line below. If we had just added MIN_MOTOR_POWER_TO_OVERCOME_FRICTION without doing this 0.9 and 1.0 become the same motor power
                motorPowers[i] += MIN_MOTOR_POWER_TO_OVERCOME_FRICTION * Math.signum(motorPowers[i]);
                motorPriorities.get(i).setTargetPower(motorPowers[i]);
            }
        }*/

        /*if((breakFollowing)
                && (Math.abs(estimate.getX() - targetPose.getX()) < xThreshold)
                && (Math.abs(estimate.getY() - targetPose.getY()) < yThreshold)
                && (Math.abs(estimate.getHeading() - targetPose.getHeading()) < headingThreshold)) {
            breakFollowing();
            setMotorPowers(0,0,0,0);
        }*/
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

    public void drive(Gamepad gamepad) {
        double forward = 0.45 * Math.tan(((gamepad.left_stick_y * -1) / 0.85));
        double turn = gamepad.right_stick_x;

        double p1 = forward + turn;
        double p2 = forward + turn;
        double p3 = forward - turn;
        double p4 = forward - turn;
        setMotorPowers(p1, p2, p3, p4);
    }

    boolean breakFollowing = false;
    Pose2d targetPose = new Pose2d(0, 0, 0);
    double xThreshold = 0.5;
    double yThreshold = 0.5;
    double headingThreshold = Math.toRadians(5.0);

    public void setBreakFollowingThresholds(Pose2d thresholds, Pose2d targetPose) {
        this.targetPose = targetPose;
        breakFollowing = true;
        xThreshold = thresholds.getX();
        yThreshold = thresholds.getY();
        headingThreshold = thresholds.getHeading();
    }

//public void breakFollowing() {
//    currentSplineToFollow.points.clear();
//}

    public Pose2d getPoseEstimate() {
        return localizer.getPoseEstimate();
    }

//public void setSpline(Spline spline) {
//    currentSplineToFollow = spline;
//}

    public void setPoseEstimate(Pose2d pose2d) {
        localizer.setPoseEstimate(pose2d);
    }

    public boolean isBusy() {
        return !doNotMove;
    }

    public Vector2 lineCircleIntersection(Pose2d start, Pose2d end, Pose2d robot, double radius) {
        //https://stackoverflow.com/questions/1073336/circle-line-segment-collision-detection-algorithm/1084899#1084899

        Vector2 direction = new Vector2(end.x - start.x, end.y - start.y);
        Vector2 robot2start = new Vector2(start.x - robot.x, start.y - robot.y);

        double a = Vector2.dot(direction, direction);
        double b = 2 * Vector2.dot(robot2start, direction);

        double c = Vector2.dot(robot2start, robot2start) - radius * radius;

        double discriminant = b * b - 4 * a * c;

        if (discriminant < 0) {
            return null;
        } else {
            discriminant = Math.sqrt(discriminant);
            double t1 = (-b + discriminant) / 2;
            double t2 = (-b - discriminant) / 2;

            if ((t1 >= 0) && (t1 <= 1)) {
                direction.mul(t1);
                return Vector2.add(direction, new Vector2(start.x, start.y));
            }
            if ((t2 >= 0) && (t2 <= 1)) {
                direction.mul(t2);
                return Vector2.add(direction, new Vector2(start.x, start.y));
            } else {
                return null;
            }

        }


    }

}
