
package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.ApriltagManager;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.util.ClusterData;

import java.util.function.Consumer;

@Autonomous(name = "driveShoot", group = "Comp")
public class driveShoot extends LinearOpMode {

    private MecanumDrive drive;
    private ApriltagManager apriltag;
    private Intake intake;
    private Shooter shooter;

    private double commandDegrees = 0.0;

    private Consumer<Double> degreesComsumer =
            val -> commandDegrees = val;

    private FtcDashboard dash = FtcDashboard.getInstance();

    // DRIVE SETTINGS - INCHES
    public static double DRIVE_DISTANCE_INCHES = 24.0;
    public static double DRIVE_POWER = 0.35;

    // Safety timeout only; distance controls the drive.
    public static double DRIVE_TIMEOUT = 8.0;

    // SHOOTING SETTINGS
    public static double TAG_WAIT_TIME = 8.0;
    public static double SPINUP_TIME = 5.0;
    public static double WAIT_BEFORE_FEEDING = 2.0;
    public static double SHOOT_TIME = 5.0;

    @Override
    public void runOpMode() {

        dash.clearTelemetry();

        // Initialize drive
        drive = new MecanumDrive(
                hardwareMap,
                new Pose2d(0, 0, 0)
        );

        // Red alliance AprilTag manager
        apriltag = new ApriltagManager(
                hardwareMap,
                "r",
                dash
        );

        // Initialize intake and shooter using the same
        // subsystems as Red Teleop
        intake = new Intake(hardwareMap);

        shooter = new Shooter(
                hardwareMap,
                apriltag,
                degreesComsumer,
                dash
        );

        telemetry.addLine("RED DRIVE SHOOT AUTON");
        telemetry.addData("Drive Distance", DRIVE_DISTANCE_INCHES);
        telemetry.addLine("READY");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            stopEverything();
            return;
        }

        // --------------------------------
        // STEP 1: DRIVE FORWARD IN INCHES
        // --------------------------------

        telemetry.addLine("DRIVING FORWARD");
        telemetry.update();

        boolean reachedDistance =
                driveInches(-DRIVE_POWER, DRIVE_DISTANCE_INCHES);

        stopDrive();

        if (!opModeIsActive()) {
            stopEverything();
            return;
        }

        if (!reachedDistance) {
            telemetry.addLine("DRIVE DISTANCE NOT REACHED");
            telemetry.addLine("STOPPING AUTON");
            telemetry.update();
            stopEverything();
            return;
        }

        // --------------------------------
        // STEP 2: SEARCH FOR A RED CLUSTER
        // --------------------------------

        telemetry.addLine("SEARCHING FOR RED CLUSTER");
        telemetry.update();

        ElapsedTime tagTimer = new ElapsedTime();
        boolean foundRedTag = false;

        while (opModeIsActive()
                && tagTimer.seconds() < TAG_WAIT_TIME) {

            apriltag.getClusters();

            ClusterData data = apriltag.getLatestData();

            if (data != null
                    && data.metadata != null
                    && data.metadata.name != null
                    && data.metadata.name.toLowerCase().startsWith("r")) {

                foundRedTag = true;
                break;
            }

            telemetry.addLine("WAITING FOR RED APRILTAG");
            telemetry.addData("Search Time", tagTimer.seconds());
            telemetry.update();

            idle();
        }

        if (!opModeIsActive()) {
            stopEverything();
            return;
        }

        // --------------------------------
        // STEP 3: ONLY SHOOT IF RED TAG FOUND
        // --------------------------------

        if (!foundRedTag) {

            telemetry.addLine("NO RED CLUSTER FOUND");
            telemetry.addLine("SHOOTING CANCELLED");
            telemetry.update();

        } else {

            // --------------------------------
            // STEP 4: SPIN UP USING RED TELEOP
            // SHOOTER LOGIC
            // --------------------------------

            telemetry.addLine("RED CLUSTER FOUND");
            telemetry.addLine("SPINNING UP");
            telemetry.update();

            ElapsedTime shooterTimer = new ElapsedTime();

            while (opModeIsActive()
                    && shooterTimer.seconds() < SPINUP_TIME) {

                apriltag.getClusters();

                shooter.setState(Shooter.State.Spin_Up);
                shooter.runState();

                telemetry.addLine("SPINNING UP SHOOTER");
                telemetry.addData("Spin Up Time", shooterTimer.seconds());
                telemetry.update();

                idle();
            }

            // --------------------------------
            // STEP 5: WAIT BEFORE FEEDING
            // KEEP SHOOTER SPINNING
            // --------------------------------

            shooterTimer.reset();

            while (opModeIsActive()
                    && shooterTimer.seconds() < WAIT_BEFORE_FEEDING) {

                apriltag.getClusters();

                shooter.setState(Shooter.State.Spin_Up);
                shooter.runState();

                telemetry.addLine("WAITING BEFORE FEEDING");
                telemetry.update();

                idle();
            }

            // --------------------------------
            // STEP 6: SHOOT USING RED TELEOP
            // LOGIC
            // --------------------------------

            if (opModeIsActive()) {

                intake.start();

                shooterTimer.reset();

                while (opModeIsActive()
                        && shooterTimer.seconds() < SHOOT_TIME) {

                    apriltag.getClusters();

                    shooter.setState(Shooter.State.Shoot);
                    shooter.runState();

                    telemetry.addLine("SHOOTING");
                    telemetry.addData("Shoot Time", shooterTimer.seconds());
                    telemetry.update();

                    idle();
                }
            }
        }

        // --------------------------------
        // STEP 7: STOP SHOOTER AND INTAKE
        // --------------------------------

        shooter.setState(Shooter.State.None);
        shooter.runState();

        intake.stop();

        if (!opModeIsActive()) {
            stopDrive();
            return;
        }

        // --------------------------------
        // STEP 8: RETURN TO START
        // --------------------------------

        telemetry.addLine("RETURNING TO START");
        telemetry.update();

        boolean returned =
                driveInches(DRIVE_POWER, DRIVE_DISTANCE_INCHES);

        stopEverything();

        telemetry.addData("Return Complete", returned);
        telemetry.addLine("AUTON COMPLETE");
        telemetry.update();
    }

    // Drive a measured distance along the robot's X axis.
    // Negative power is forward, matching the previous
    // driveShoot direction.
    private boolean driveInches(double power, double inches) {

        drive.localizer.update();

        double startX = drive.localizer.getPose().position.x;

        ElapsedTime safetyTimer = new ElapsedTime();

        while (opModeIsActive()) {

            drive.localizer.update();

            double currentX = drive.localizer.getPose().position.x;
            double traveled = Math.abs(currentX - startX);

            if (traveled >= inches) {
                stopDrive();
                return true;
            }

            if (safetyTimer.seconds() >= DRIVE_TIMEOUT) {
                stopDrive();
                return false;
            }

            drive.setDrivePowers(
                    new PoseVelocity2d(
                            new Vector2d(power, 0),
                            0
                    )
            );

            telemetry.addData("Target Inches", inches);
            telemetry.addData("Travelled Inches", traveled);
            telemetry.addData("Drive Power", power);
            telemetry.update();

            idle();
        }

        stopDrive();
        return false;
    }

    private void stopDrive() {

        drive.setDrivePowers(
                new PoseVelocity2d(
                        new Vector2d(0, 0),
                        0
                )
        );
    }

    private void stopEverything() {

        if (shooter != null) {
            shooter.setState(Shooter.State.None);
            shooter.runState();
        }

        if (intake != null) {
            intake.stop();
        }

        if (drive != null) {
            stopDrive();
        }
    }
}
