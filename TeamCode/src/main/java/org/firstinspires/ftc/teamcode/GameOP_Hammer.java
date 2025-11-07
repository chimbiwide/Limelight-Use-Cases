/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.Random;


@TeleOp(name="Game: Hammer 0", group="Linear OpMode")

public class GameOP_Hammer extends LinearOpMode {

    public static final double GAMETIME = 120;

    private double gowtham_speed = 0.6;

    private double hammerpos = .5;
    private final double HAMMER_MAX = 1; //todo DO NOT SEt tthis To 000000 !!!!!!
    private final double HAMMER_MIN = 0.5; //todo DO NOT set this TO 11111!!!!!!!
    private static final int DISABLED_TIME = 5;

    private static int shields = 0;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private Servo hammer = null;

    private TouchSensor touch_l;
    private TouchSensor touch_r;
    private boolean disabled = false;


    //disabled time

    private int shield = 0;
    private double disabled_until = 0;

    private int score = 10;

    @Override
    public void runOpMode() {


        leftDrive = hardwareMap.get(DcMotor.class, "left");
        rightDrive = hardwareMap.get(DcMotor.class, "right");
        hammer = hardwareMap.get(Servo.class, "hammer");
        touch_r = hardwareMap.get(TouchSensor.class, "touch_r");
        touch_l = hardwareMap.get(TouchSensor.class, "touch_l");

        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        leftDrive.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        while (opModeIsActive() && (runtime.seconds() < GAMETIME)) {
            if ((touch_r.isPressed() || touch_l.isPressed()) && !disabled) {
                if (shield > 0) {
                    shield --;
                }
                else {
                    disabled_until = (runtime.seconds() + DISABLED_TIME);
                    score -= 5;
                }
            }
            disabled = (disabled_until > runtime.seconds());

            if (!disabled) {
                double drive = gamepad1.left_stick_y;
                double turn = gamepad1.right_stick_x;
                double rightPower = (drive + turn) * gowtham_speed;
                double leftPower = (drive - turn) * gowtham_speed;

                leftDrive.setPower(leftPower);
                rightDrive.setPower(rightPower);

                //hammer.setPosition(gamepad1.right_trigger * (HAMMER_MAX - HAMMER_MIN) + HAMMER_MIN);
                if (gamepad1.right_bumper) {
                    hammerpos += 0.05;
                } else if (gamepad1.left_bumper) {
                    hammerpos -= 0.05;
                }

                if (hammerpos < HAMMER_MIN) {
                    hammerpos = HAMMER_MIN;
                } else if (hammerpos > HAMMER_MAX) {
                    hammerpos = HAMMER_MAX;
                }

                hammer.setPosition(hammerpos);

            }
            else {
                leftDrive.setPower(0);
                rightDrive.setPower(0);
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Run Time", runtime.seconds());
            telemetry.addData("Time Remaining", GAMETIME - runtime.seconds());
            telemetry.addData("Gowtham Speed", gowtham_speed);
            telemetry.addData("Hammer Position", hammer.getPosition() + ", " + hammerpos);
            telemetry.addData("Disabled", disabled);
            telemetry.addData("Score: ", score);
            telemetry.update();
        }
    }
}
