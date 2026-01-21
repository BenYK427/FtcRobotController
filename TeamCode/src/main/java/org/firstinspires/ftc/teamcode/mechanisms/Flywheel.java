package org.firstinspires.ftc.teamcode.mechanisms;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Flywheel {

    private DcMotorEx motorFlywheel;
    private DcMotor motorFlywheel2;


    public void init(HardwareMap hwMap) {
        motorFlywheel = hwMap.get(DcMotorEx.class, "motorFlywheel");
        motorFlywheel2 = hwMap.get(DcMotor.class, "motorFlywheel2");

        motorFlywheel.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFlywheel2.setDirection(DcMotorSimple.Direction.REVERSE);

        motorFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void flywheel(double distance, boolean autoFlywheel){
        if(autoFlywheel){
            motorFlywheel.setVelocity(1500);
            motorFlywheel2.setPower(motorFlywheel.getPower());
        }

    }
}
