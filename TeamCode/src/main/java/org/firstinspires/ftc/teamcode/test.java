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

@TeleOp(name="Flywheel", group="Starterbot")

public class test extends LinearOpMode {

    // Declare OpMode members
    private DcMotor motor;

    double flywheelPower = 1.0;
    /*
     * These constants control the catapult's wind and release speeds.
     * Adjust as needed.
     */

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        motor = hardwareMap.dcMotor.get("shoot");

        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //Drive Control
            if (gamepad1.right_bumper) {
                motor.setPower(flywheelPower);
            }
            else if (gamepad1.left_bumper) {
                motor.setPower(-flywheelPower);
            }
            motor.setPower(0);
        }
    }
}