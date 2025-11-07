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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="Hard: Linear OpMode", group="Linear OpMode")

public class HardOpMode_Linear extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    private double time_remaining = 0;

    private boolean dsiabled = false;

    private double gowtham_speed = 0.7;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftDrive  = hardwareMap.get(DcMotor.class, "left");
        rightDrive = hardwareMap.get(DcMotor.class, "right");
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        leftDrive.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();
        runtime.reset();
        boolean pright_bumper = false;
        boolean pleft_bumper = false;
        boolean pcircle = false;
        boolean canDrive = true;
        // run until the end of the match (driver presses STOP)
        while (runtime.seconds() <= 120 && opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            if (gamepad1.right_bumper && !pright_bumper) {
                gowtham_speed += 0.1;
                gamepad1.rumble(0, 10, 300);
                if (gowtham_speed > 0.7) {
                    gowtham_speed = 0.7;
                }
            }
            else if (gamepad1.left_bumper && !pleft_bumper) {
                gowtham_speed -= 0.1;
                gamepad1.rumble(10,0,300);
                if (gowtham_speed < 0 ) {
                    gowtham_speed = 0;
                }
            }
            if (gamepad1.circle) {
                canDrive = false;
                gamepad1.rumble(1,1,200);
            }
            else if (!gamepad1.circle && pcircle) {
                canDrive = true;
                gamepad1.rumble(1,1,100);
            }

            pright_bumper = gamepad1.right_bumper;
            pleft_bumper = gamepad1.left_bumper;
            pcircle = gamepad1.circle;


            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;
            rightPower    = Range.clip(drive + turn, -gowtham_speed, gowtham_speed) ;
            leftPower   = Range.clip(drive - turn, -gowtham_speed, gowtham_speed) ;
            leftDrive.setPower(leftPower * 0.9);
            rightDrive.setPower(rightPower);

            time_remaining = (double) (120 - runtime.seconds());

            // Show the elapsed game time and wheel power.
            telemetry.addData("Run Time", runtime.seconds());
            telemetry.addData("Time Remaining", time_remaining);
            telemetry.addData("Gowtham Speed", gowtham_speed);
            telemetry.addData("Time Score", (int)((time_remaining / 3)+0.5));
            telemetry.update();
        }
    }
}
