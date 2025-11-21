package org.firstinspires.ftc.teamcode.config;

import static org.firstinspires.ftc.teamcode.config.Constants.BACK_LEFT;
import static org.firstinspires.ftc.teamcode.config.Constants.BACK_RIGHT;
import static org.firstinspires.ftc.teamcode.config.Constants.FRONT_LEFT;
import static org.firstinspires.ftc.teamcode.config.Constants.FRONT_RIGHT;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HardwareMapper {
    public static DcMotor[] getMotors(HardwareMap hardwaremap) {
        DcMotor[] motors = new DcMotor[4];
        motors[FRONT_LEFT] = hardwaremap.get(DcMotor.class, "frontleft");
        motors[BACK_LEFT] = hardwaremap.get(DcMotor.class, "backleft");
        motors[FRONT_RIGHT] = hardwaremap.get(DcMotor.class, "frontright");
        motors[BACK_RIGHT] = hardwaremap.get(DcMotor.class, "backright");

        motors[FRONT_LEFT].setDirection(DcMotor.Direction.REVERSE);
        return motors;
    }
}
