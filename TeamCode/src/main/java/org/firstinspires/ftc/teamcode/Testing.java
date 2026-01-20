package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.mechanisms.DcMotorTest;

@Disabled
public class Testing extends  OpMode{
    DcMotorTest motor = new DcMotorTest();

    public void init(){
        motor.init(hardwareMap);

    }

    @Override
    public void loop() {
        motor.setIntakeMotorSpeed(0.5);
        motor.setMotorFlywheelSpeed(0.2);
    }
}
