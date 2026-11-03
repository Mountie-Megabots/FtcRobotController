package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.ArrayList;
import java.util.List;

/**
 * High-performance, zero-allocation Field-Centric Path Follower for FTC Mecanum Robots.
 * Works natively with Pose2D from goBILDA Pinpoint, SparkFun OTOS, or standard FTC odometry.
 */
public class SimplePathFollower {

    /**
     * Waypoint representation.
     */
    public static class Waypoint {
        public final double x;           // Inches
        public final double y;           // Inches
        public final double headingRad;  // Radians
        public final double maxPower;    // 0.0 to 1.0
        public final double passRadius;  // Inches: distance threshold to advance to next waypoint
        public final boolean stopAtEnd;  // If true, decelerate to a stop at this waypoint

        public Waypoint(double x, double y, double headingDeg, double maxPower, double passRadius, boolean stopAtEnd) {
            this.x = x;
            this.y = y;
            this.headingRad = Math.toRadians(headingDeg);
            this.maxPower = Math.max(0.05, Math.min(1.0, maxPower));
            this.passRadius = passRadius;
            this.stopAtEnd = stopAtEnd;
        }

        // Convenience constructors
        public Waypoint(double x, double y, double headingDeg, double maxPower) {
            this(x, y, headingDeg, maxPower, 3.0, false);
        }

        public Waypoint(double x, double y, double headingDeg) {
            this(x, y, headingDeg, 0.8, 3.0, false);
        }
    }

    // Gains & Tuning Parameters
    private double kPTranslation = 0.08;   // Power scaling per inch of position error
    private double kPHeading = 0.8;        // Power scaling per radian of heading error
    private double slowdownDistance = 8.0; // Distance (inches) to start decelerating for stop waypoints
    private double posTolerance = 0.75;    // Acceptable position error (inches)
    private double headingToleranceRad = Math.toRadians(2.0); // Acceptable heading error (radians)

    // Path Execution State
    private final List<Waypoint> path = new ArrayList<>();
    private int currentTargetIdx = 0;
    private boolean isFinished = false;

    // Pre-allocated power array to prevent Garbage Collection (GC) in loop [LF, RF, LB, RB]
    private final double[] motorPowers = new double[4];

    public SimplePathFollower() {}

    /** Configures proportional controller gains. */
    public SimplePathFollower setGains(double kPTranslation, double kPHeading) {
        this.kPTranslation = kPTranslation;
        this.kPHeading = kPHeading;
        return this;
    }

    /** Configures target tolerances and slowdown distance. */
    public SimplePathFollower setTolerances(double posToleranceInches, double headingToleranceDeg, double slowdownDistInches) {
        this.posTolerance = posToleranceInches;
        this.headingToleranceRad = Math.toRadians(headingToleranceDeg);
        this.slowdownDistance = slowdownDistInches;
        return this;
    }

    // Fluent API to build paths
    public SimplePathFollower addWaypoint(Waypoint waypoint) {
        path.add(waypoint);
        return this;
    }

    public SimplePathFollower addWaypoint(double x, double y, double headingDeg) {
        path.add(new Waypoint(x, y, headingDeg));
        return this;
    }

    public SimplePathFollower addWaypoint(double x, double y, double headingDeg, double maxPower) {
        path.add(new Waypoint(x, y, headingDeg, maxPower));
        return this;
    }

    public SimplePathFollower addWaypoint(double x, double y, double headingDeg, double maxPower, boolean stopAtEnd) {
        path.add(new Waypoint(x, y, headingDeg, maxPower, 3.0, stopAtEnd));
        return this;
    }

    /** Prepares the follower to execute the loaded path. */
    public void start() {
        currentTargetIdx = 0;
        isFinished = path.isEmpty();
    }

    /** Clears existing path points. */
    public void clearPath() {
        path.clear();
        currentTargetIdx = 0;
        isFinished = true;
    }

    /**
     * Call this inside your main OpMode loop iteration.
     *
     * @param currentPose Current robot pose (from Pinpoint, OTOS, or odometry)
     * @return Array of double motor powers [LF, RF, LB, RB] normalized to <= 1.0
     */
    public double[] update(Pose2D currentPose) {
        if (isFinished || path.isEmpty()) {
            motorPowers[0] = 0; motorPowers[1] = 0;
            motorPowers[2] = 0; motorPowers[3] = 0;
            return motorPowers;
        }

        Waypoint target = path.get(currentTargetIdx);
        double robotX = currentPose.getX(DistanceUnit.INCH);
        double robotY = currentPose.getY(DistanceUnit.INCH);
        double robotHeading = currentPose.getHeading(AngleUnit.RADIANS);

        // Compute field-centric position error
        double deltaXField = target.x - robotX;
        double deltaYField = target.y - robotY;
        double distanceToTarget = Math.hypot(deltaXField, deltaYField);

        // Compute heading error wrapped between [-PI, PI]
        double headingError = wrapAngleRad(target.headingRad - robotHeading);

        boolean isLastWaypoint = (currentTargetIdx == path.size() - 1);
        boolean isStopWaypoint = target.stopAtEnd || isLastWaypoint;

        // Waypoint advancement logic
        if (!isStopWaypoint) {
            // Pass-through waypoint transition
            if (distanceToTarget <= target.passRadius) {
                currentTargetIdx++;
                return update(currentPose);
            }
        } else {
            // Terminal waypoint transition
            if (distanceToTarget <= posTolerance && Math.abs(headingError) <= headingToleranceRad) {
                if (isLastWaypoint) {
                    isFinished = true;
                    motorPowers[0] = 0; motorPowers[1] = 0;
                    motorPowers[2] = 0; motorPowers[3] = 0;
                    return motorPowers;
                } else {
                    currentTargetIdx++;
                    return update(currentPose);
                }
            }
        }

        // Field-centric to Robot-centric vector rotation
        double cosHeading = Math.cos(robotHeading);
        double sinHeading = Math.sin(robotHeading);

        double forwardError = deltaXField * cosHeading + deltaYField * sinHeading;
        double strafeError  = -deltaXField * sinHeading + deltaYField * cosHeading;

        // Apply smooth slowdown profile near stop waypoints
        double driveScale = 1.0;
        if (isStopWaypoint && distanceToTarget < slowdownDistance) {
            driveScale = Math.max(0.1, distanceToTarget / slowdownDistance);
        }

        double driveForward = forwardError * kPTranslation * driveScale;
        double driveStrafe  = strafeError  * kPTranslation * driveScale;
        double driveTurn    = headingError  * kPHeading;

        // Cap translation vector to waypoint's max power setting
        double transMagnitude = Math.hypot(driveForward, driveStrafe);
        if (transMagnitude > target.maxPower) {
            double scale = target.maxPower / transMagnitude;
            driveForward *= scale;
            driveStrafe *= scale;
        }

        // Calculate standard Mecanum motor powers
        double lf = driveForward + driveStrafe + driveTurn;
        double rf = driveForward - driveStrafe - driveTurn;
        double lb = driveForward - driveStrafe + driveTurn;
        double rb = driveForward + driveStrafe - driveTurn;

        // Normalize powers so no motor exceeds 1.0
        double maxPower = Math.max(Math.abs(lf), Math.max(Math.abs(rf),
                Math.max(Math.abs(lb), Math.abs(rb))));
        if (maxPower > 1.0) {
            lf /= maxPower;
            rf /= maxPower;
            lb /= maxPower;
            rb /= maxPower;
        }

        motorPowers[0] = lf;
        motorPowers[1] = rf;
        motorPowers[2] = lb;
        motorPowers[3] = rb;

        return motorPowers;
    }

    public boolean isFinished() {
        return isFinished;
    }

    public int getCurrentWaypointIndex() {
        return currentTargetIdx;
    }

    private static double wrapAngleRad(double angleRad) {
        while (angleRad > Math.PI) angleRad -= 2.0 * Math.PI;
        while (angleRad < -Math.PI) angleRad += 2.0 * Math.PI;
        return angleRad;
    }
}