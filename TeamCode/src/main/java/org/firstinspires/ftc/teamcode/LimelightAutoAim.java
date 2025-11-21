package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/**
 * Autonomous OpMode that:
 * 1. Moves forward for 2 seconds
 * 2. Uses Limelight target X to auto-aim at AprilTag
 * 3. Shoots the catapult
 */
@Autonomous(name = "Scrimmage Auto", group = "Auto")
public class LimelightAutoAim extends LinearOpMode {

    // Hardware
    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private DcMotor motorCatapult;
    private Limelight3A limelight;
    private IMU imu;

    private ElapsedTime runtime = new ElapsedTime();

    // Catapult constants
    private static final double WIND = 1.0;
    private static final double RELEASE = -1.0;

    // Auto-aim constants
    private static final double TURN_POWER = 0.3;           // Power for turning during aim
    private static final double TARGET_X_TOLERANCE = 2.0;   // Acceptable error in degrees
    private static final double MAX_AIM_TIME = 1.0;         // Max time to spend aiming (seconds)

    // Pipeline selection (change based on which tag you want to target)
    // Pipeline 7: Tags 21, 22, 23 (OBELISK)
    // Pipeline 8: Tag 20 (Blue GOAL)
    // Pipeline 9: Tag 24 (Red GOAL)
    private int pipeline = 8;  // Default to Blue GOAL

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        // Initialize hardware
        initializeHardware();

        // Wait for start
        telemetry.addData("Status", "Ready to start");
        telemetry.addData("Pipeline", pipeline);
        telemetry.update();

        waitForStart();
        runtime.reset();

        if (opModeIsActive()) {
            // Step 1: Back up for 1 second
            telemetry.addData("Step", "1. Backing up");
            telemetry.update();
            driveBackward(1.5);

            // Step 2: Auto-aim to AprilTag using Limelight
            telemetry.addData("Step", "2. Auto-aiming");
            telemetry.update();
            autoAimToTarget();

            // Step 3: Shoot
            telemetry.addData("Step", "3. Shooting");
            telemetry.update();
            launch();

            // Done
            telemetry.addData("Status", "Complete!");
            telemetry.update();
        }
    }

    /**
     * Initialize all hardware components
     */
    private void initializeHardware() {
        // Initialize drive motors
        leftDrive = hardwareMap.dcMotor.get("left");
        rightDrive = hardwareMap.dcMotor.get("right");
        motorCatapult = hardwareMap.dcMotor.get("shoot");

        // Set motor directions
        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        motorCatapult.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set motors to run without encoders for time-based driving
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(pipeline);
        limelight.start();

        // Initialize IMU
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );
        imu.initialize(new IMU.Parameters(orientation));
    }

    /**
     * Drive forward at 0.5 power for the specified duration
     * @param seconds Duration to drive forward
     */
    private void driveForward(double seconds) {
        runtime.reset();

        while (opModeIsActive() && runtime.seconds() < seconds) {
            leftDrive.setPower(0.5);
            rightDrive.setPower(0.5);

            telemetry.addData("Driving", "%.1f / %.1f sec", runtime.seconds(), seconds);
            telemetry.update();
        }

        // Stop motors
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        sleep(250);
    }

    /**
     * Drive backward at 0.5 power for the specified duration
     * @param seconds Duration to drive backward
     */
    private void driveBackward(double seconds) {
        runtime.reset();

        while (opModeIsActive() && runtime.seconds() < seconds) {
            leftDrive.setPower(-0.5);
            rightDrive.setPower(-0.5);

            telemetry.addData("Backing up", "%.1f / %.1f sec", runtime.seconds(), seconds);
            telemetry.update();
        }

        // Stop motors
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        sleep(250);
    }

    /**
     * Turn slightly to the right for the specified duration
     * Use negative duration to turn left instead
     * @param seconds Duration to turn (positive = right, negative = left)
     */
    private void turnSlightly(double seconds) {
        runtime.reset();
        double turnDuration = Math.abs(seconds);
        boolean turnRight = seconds > 0;

        while (opModeIsActive() && runtime.seconds() < turnDuration) {
            if (turnRight) {
                // Turn right (clockwise)
                leftDrive.setPower(TURN_POWER);
                rightDrive.setPower(-TURN_POWER);
            } else {
                // Turn left (counter-clockwise)
                leftDrive.setPower(-TURN_POWER);
                rightDrive.setPower(TURN_POWER);
            }

            telemetry.addData("Turning", "%s %.1f / %.1f sec",
                turnRight ? "Right" : "Left",
                runtime.seconds(),
                turnDuration);
            telemetry.update();
        }

        // Stop motors
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        sleep(250);
    }

    /**
     * Auto-aim to AprilTag using Limelight's target X value
     * Adjusts robot rotation until target is centered
     */
    private void autoAimToTarget() {
        runtime.reset();
        boolean targetAcquired = false;

        while (opModeIsActive() && runtime.seconds() < MAX_AIM_TIME && !targetAcquired) {
            // Update robot orientation for Limelight
            YawPitchRollAngles orientation = AprilTag.getOrientation(imu);
            limelight.updateRobotOrientation(orientation.getYaw());

            // Get latest Limelight result
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                double tx = result.getTx();  // Horizontal offset in degrees

                telemetry.addData("Target X", "%.2f degrees", tx);
                telemetry.addData("Status", "Target found, aiming...");

                // Check if target is centered (within tolerance)
                if (Math.abs(tx) <= TARGET_X_TOLERANCE) {
                    // Target is centered
                    leftDrive.setPower(0);
                    rightDrive.setPower(0);
                    targetAcquired = true;
                    telemetry.addData("Status", "Target locked!");
                } else {
                    // Turn to center the target
                    // If tx is positive, target is to the right -> turn right
                    // If tx is negative, target is to the left -> turn left
                    if (tx > 0) {
                        // Turn right (clockwise)
                        leftDrive.setPower(TURN_POWER);
                        rightDrive.setPower(-TURN_POWER);
                    } else {
                        // Turn left (counter-clockwise)
                        leftDrive.setPower(-TURN_POWER);
                        rightDrive.setPower(TURN_POWER);
                    }
                }
            } else {
                // No target detected
                leftDrive.setPower(0);
                rightDrive.setPower(0);
                telemetry.addData("Status", "No target detected");
            }

            telemetry.update();
        }

        // Stop motors
        leftDrive.setPower(0);
        rightDrive.setPower(0);

        if (targetAcquired) {
            telemetry.addData("Aim Result", "Success - Target locked");
        } else {
            telemetry.addData("Aim Result", "Timeout - Target not centered");
        }
        telemetry.update();
        sleep(500);
    }

    /**
     * Launch the catapult to shoot
     */
    private void launch() {
        if (opModeIsActive()) {
            sleep(500);
            motorCatapult.setPower(RELEASE);
            sleep(2000);
            motorCatapult.setPower(WIND);
            sleep(2000);
            motorCatapult.setPower(RELEASE);
            sleep(2000);
            motorCatapult.setPower(WIND);
            sleep(2000);
            motorCatapult.setPower(RELEASE);
            sleep(2000);
            motorCatapult.setPower(WIND);
            sleep(2000);
            motorCatapult.setPower(0);

            telemetry.addData("Catapult", "Fired!");
            telemetry.update();
        }
    }
}
