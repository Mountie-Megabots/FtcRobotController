package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Autonomous(name = "Simple Path Following Auto", group = "Autonomous")
public class SimplePathAuto extends LinearOpMode {

    private DcMotor leftFront, rightFront, leftBack, rightBack;
    private GoBildaPinpointDriver pinpoint;
    private SimplePathFollower pathFollower;

    @Override
    public void runOpMode() {
        // 1. Initialize Motors
        leftFront  = hardwareMap.get(DcMotor.class, "front-left");
        rightFront = hardwareMap.get(DcMotor.class, "front-right");
        leftBack   = hardwareMap.get(DcMotor.class, "back-left");
        rightBack  = hardwareMap.get(DcMotor.class, "back-right");

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        // 2. Initialize Pinpoint / Localization Sensor
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.resetPosAndIMU();

        // 3. Define the Autonomous Path for Students
        pathFollower = new SimplePathFollower();
        pathFollower
                // Waypoint 1: Drive forward to (0, 24 inches) facing 0 deg at 80% speed (pass-through)
                .addWaypoint(0, 24, 0, 0.8);


        telemetry.addData("Status", "Initialized. Ready for Start.");
        telemetry.update();

        waitForStart();

        pathFollower.start();

        // 4. Autonomous Control Loop
        while (opModeIsActive() && !pathFollower.isFinished()) {
            pinpoint.update();
            Pose2D currentPose = pinpoint.getPosition();

            // Calculate motor powers
            double[] powers = pathFollower.update(currentPose);

            // Set motor speeds
            leftFront.setPower(powers[0]);
            rightFront.setPower(powers[1]);
            leftBack.setPower(powers[2]);
            rightBack.setPower(powers[3]);

            // Telemetry feedback
            telemetry.addData("Target Waypoint", pathFollower.getCurrentWaypointIndex());
            telemetry.addData("Target Waypoint-X", pathFollower.getWaypoint(pathFollower.getCurrentWaypointIndex()).x);
            telemetry.addData("Target Waypoint-y", pathFollower.getWaypoint(pathFollower.getCurrentWaypointIndex()).y);
            telemetry.addData("Robot X (in)", currentPose.getX(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH));
            telemetry.addData("Robot Y (in)", currentPose.getY(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH));
            telemetry.addData("Robot Heading (deg)", currentPose.getHeading(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES));
            telemetry.update();
        }

        // Stop motors when path completes
        leftFront.setPower(0); rightFront.setPower(0);
        leftBack.setPower(0);  rightBack.setPower(0);
    }
}