
package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

@Autonomous(name = "Shoot Only", group = "Comp")
public class ShootOnly extends LinearOpMode {

    private DcMotorEx shooterLeft;
    private DcMotorEx shooterRight;
    private DcMotorEx indexer;

    private Intake intake;

    // Adjustable times
    public static double SPINUP_TIME = 5.0;
    public static double WAIT_BEFORE_FEEDING = 2.0;
    public static double SHOOT_TIME = 5.0;

    @Override
    public void runOpMode() {

        // Get motors using the exact names from your project
        shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");
        indexer = hardwareMap.get(DcMotorEx.class, "indexer");

        // Use your actual Intake subsystem
        intake = new Intake(hardwareMap);

        // Match Shooter.java motor configuration
        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterRight.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Match Shooter.java indexer configuration
        indexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        indexer.setDirection(DcMotorSimple.Direction.REVERSE);
        indexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Match the shooter PIDF settings from Shooter.java
        PIDFCoefficients gains = new PIDFCoefficients(26, 2, 8, 10);

        shooterLeft.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER, gains);

        shooterRight.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER, gains);

        // Start everything stopped
        shooterLeft.setPower(0);
        shooterRight.setPower(0);
        indexer.setPower(0);
        intake.stop();

        telemetry.addLine("SHOOT ONLY");
        telemetry.addLine("READY");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // STEP 1: Spin up both shooter motors
        shooterLeft.setPower(.25);
        shooterRight.setPower(.25);

        double startTime = getRuntime();

        while (opModeIsActive()
                && getRuntime() - startTime < SPINUP_TIME) {

            telemetry.addLine("SPINNING UP SHOOTER");
            telemetry.addData("Left Shooter", shooterLeft.getVelocity());
            telemetry.addData("Right Shooter", shooterRight.getVelocity());
            telemetry.update();

            idle();
        }

        // STEP 2: Wait before feeding
        if (opModeIsActive()) {

            telemetry.addLine("WAITING BEFORE FEEDING");
            telemetry.update();

            sleep((long)(WAIT_BEFORE_FEEDING * 1000));
        }

        // STEP 3: Start intake and indexer together
        if (opModeIsActive()) {

            intake.start();
            indexer.setPower(.75);

            startTime = getRuntime();

            while (opModeIsActive()
                    && getRuntime() - startTime < SHOOT_TIME) {

                telemetry.addLine("SHOOTING");
                telemetry.addData("Left Shooter", shooterLeft.getVelocity());
                telemetry.addData("Right Shooter", shooterRight.getVelocity());
                telemetry.addData("Indexer", indexer.getPower());
                telemetry.update();

                idle();
            }
        }

        // STEP 4: Stop everything
        intake.stop();
        indexer.setPower(0);
        shooterLeft.setPower(0);
        shooterRight.setPower(0);

        telemetry.addLine("SHOOTING COMPLETE");
        telemetry.update();
    }
}
