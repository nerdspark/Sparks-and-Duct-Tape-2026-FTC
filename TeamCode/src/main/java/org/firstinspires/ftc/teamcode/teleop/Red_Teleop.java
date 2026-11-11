package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.ApriltagManager;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.util.PID;

import java.util.function.Consumer;

@TeleOp(name = "Red Teleop", group = "Comp")
public class Red_Teleop extends OpMode {
    private MecanumDrive drive;
    private double commandDegrees = 0.0;
    private Consumer<Double> degreesComsumer = (val) -> commandDegrees = val;
    private final PID turnController = new PID(0.088, 0.1, 0.00000001);
    private ApriltagManager apriltag;
    private Intake intake;
    private Shooter shooter;
    private FtcDashboard dash = FtcDashboard.getInstance();

    public void init() {
        dash.clearTelemetry();
        drive = new MecanumDrive(hardwareMap, new Pose2d(0,0, 0));
        apriltag = new ApriltagManager(hardwareMap, "r", dash);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap, apriltag, degreesComsumer, dash);
    }

    public void loop() {
        apriltag.getClusters();

        if (gamepad1.aWasPressed()) {
            shooter.setState(Shooter.State.Align);
        }
        if (gamepad1.left_bumper) {
            shooter.setState(Shooter.State.Spin_Up);
        }
        if (gamepad1.bWasPressed()) {
            shooter.setState(Shooter.State.Shoot);
        } else if (gamepad1.bWasReleased()) {
            shooter.setState(Shooter.State.None);
        }

        shooter.runState();


        if (gamepad2.xWasPressed()) {
            intake.start();
        }
        if (gamepad2.yWasPressed()) {
            intake.stop();
        }
        if (gamepad2.bWasPressed()) {
            intake.reverse();
        } else if (gamepad2.bWasReleased()) {
            intake.stop();
        }
        if (gamepad2.leftBumperWasPressed()) {
            intake.flowerIntake();
        } else if (gamepad2.leftBumperWasReleased()) {
            intake.barUp();
        }

        // Drive
        double y = gamepad1.right_stick_y;
        double x = gamepad1.right_stick_x;

        if (gamepad1.left_stick_x > 0.15) {
            commandDegrees -= 0.5;
        } else if (gamepad1.left_stick_x < -0.15) {
            commandDegrees += 0.5;
        }

        // Auto turn with d-pad
        if(gamepad1.dpad_down) {
            if (Math.toDegrees(drive.localizer.getPose().heading.toDouble()) < 0) {
                commandDegrees = -180;
            } else {
                commandDegrees = 180;
            }
        } else if (gamepad1.dpad_up) {
            commandDegrees = 0;
        } else if (gamepad1.dpad_left) {
            commandDegrees = 90;
        } else if (gamepad1.dpad_right) {
            commandDegrees = -90;
        }

        drive.localizer.update();

        double rx = turnController.calc(drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES), commandDegrees);

        // ********************* Driving Math ***** Do NOT Change *********************
        double botHeading = drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(botHeading);
        double rotY = x * Math.sin(botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(rotY, rotX), rx));
    }
}
