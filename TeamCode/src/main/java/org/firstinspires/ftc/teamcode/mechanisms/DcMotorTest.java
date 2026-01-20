package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DcMotorTest {
    private DcMotor intakeMotor, motorFlywheel;


    public void init(HardwareMap hwMap){
        intakeMotor = hwMap.get(DcMotor.class, "intakeMotor");
        motorFlywheel = hwMap.get(DcMotor.class, "motorFlywheel");

        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFlywheel.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void setIntakeMotorSpeed(double speed){
        intakeMotor.setPower(speed);
    }
    public void setMotorFlywheelSpeed(double speed2){
        motorFlywheel.setPower(speed2);
    }

}
