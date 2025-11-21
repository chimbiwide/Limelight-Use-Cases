package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.dataType.Point;

import java.util.List;

@TeleOp(name="StarterTeleOP", group="Starterbot")

public class StarterTeleOP extends LinearOpMode {

    // Declare OpMode members.
    private Limelight3A limelight;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private DcMotor motorCatapult;

    double leftPower;
    double rightPower;

    //The following is a selection for the drive control style.
    private enum DriveStyle {
        SPLIT_ARCADE,
        ARCADE,
        TANK
    }
    private DriveStyle drive = DriveStyle.SPLIT_ARCADE;//Select drive mode

    /*
     * These constants control the catapult's wind and release speeds.
     * Adjust as needed.
     */
    private static final double WIND = 1.0;
    private static final double RELEASE = -1.0;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        /* Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app).
         */
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        int pipeline = 8;
        //pipeline 7 is ID21,22,23
        //pipeline 8 is ID20(BLUE)
        //pipeline 9 is ID24(RED)
        limelight.pipelineSwitch(pipeline);
        leftDrive = hardwareMap.dcMotor.get("left");
        rightDrive = hardwareMap.dcMotor.get("right");
        motorCatapult = hardwareMap.dcMotor.get("shoot");

        leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        motorCatapult.setDirection(DcMotorSimple.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        limelight.start();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //Drive Control
            double y = gamepad1.left_stick_y; // Note: pushing stick forward gives negative value
            double x = -gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            /* Denominator is the largest motor power (absolute value) or 1
             * This ensures all the powers maintain the same ratio, but only when
             * at least one is out of the range [-1, 1]
             */
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);


            if (drive == DriveStyle.SPLIT_ARCADE) { // split arcade drive
                leftPower = (y + rx) / denominator;
                rightPower = (y - rx) / denominator;
            } else if (drive == DriveStyle.ARCADE) { // arcade drive
                leftPower = (y + x) / denominator;
                rightPower = (y - x) / denominator;
            } else if (drive == DriveStyle.TANK) { // tank drive            
                leftPower = gamepad1.left_stick_y;
                rightPower = gamepad1.right_stick_y;
            }

            // Send calculated power to wheels
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);


            //Launch control
            if (gamepad1.left_trigger > 0.5) {
                motorCatapult.setPower(WIND);
            } else if (gamepad1.right_trigger > 0.5) {
                motorCatapult.setPower(RELEASE);
            } else{
                motorCatapult.setPower(0); //This turns the catapult motor off when not in use
            }



            // Show the elapsed game time.
            telemetry.addData("Status", "Run Time: " + runtime.toString());

            // Telemetry data for drive type
            if (drive == DriveStyle.SPLIT_ARCADE) {
                telemetry.addData("Style","SplitArcade");
            } else if (drive == DriveStyle.ARCADE) {
                telemetry.addData("Style","Arcade");
            } else if (drive == DriveStyle.TANK) {
                telemetry.addData("Style","Tank");
            }

            // Telemetry data for catapult status
            if (motorCatapult.getPower() == WIND) {
                telemetry.addData("Catapult Status","Wind");
            } else if (motorCatapult.getPower() == RELEASE) {
                telemetry.addData("Catapult Status","Release");
            } else {
                telemetry.addData("Catapult Status","Off");
            }

            //Wind Release status
            telemetry.addData("Wind: ", WIND);
            telemetry.addData("Release", RELEASE);
            LLResult llresult = limelight.getLatestResult();

            if (llresult != null && llresult.isValid()) {
                List<LLResultTypes.FiducialResult> results = llresult.getFiducialResults();
                Pose3D botpose = llresult.getBotpose();

                Point mt1_postion = AprilTag.getPosition(botpose);

                int tagID = AprilTag.getTagID(results);
                double Tx = llresult.getTx();
                double Ty = llresult.getTy();
                double Ta = llresult.getTa();

                telemetry.addData("April Tag ", tagID);
                telemetry.addData("Target x", Tx);
                telemetry.addData("Target y", Ty);
                telemetry.addData("MT1 Coordinates", mt1_postion.toString());
            }
            else {
                telemetry.addLine("No AprilTag detected");
                telemetry.addData("Pipline", pipeline);
            }

            telemetry.update();
        }
    }
}