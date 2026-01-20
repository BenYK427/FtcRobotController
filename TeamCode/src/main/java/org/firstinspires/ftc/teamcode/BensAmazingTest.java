package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.MechanumDrive;

@TeleOp
public class BensAmazingTest extends OpMode{

    MechanumDrive drive = new MechanumDrive();
    double forward, strafe, turn;
    boolean adjust;

    Intake intakeMotorPower = new Intake();
    double intakePower;

    public void init(){
        drive.init(hardwareMap);
        intakeMotorPower.init(hardwareMap);
        telemetry.addLine("init complete");

    }

    @Override
    public void loop() {


        //---------------------------Drive------------------------------
        if(gamepad1.b){
            adjust = true;
        } else {
            adjust = false;
        }

        forward = gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x*-1.1;
        turn = gamepad1.right_stick_x*-0.4;

        drive.drive(forward, strafe, turn, adjust);


        //--------------------------Intake--------------------

        intakePower = gamepad1.left_stick_y;

        intakeMotorPower.IntakeMotorPower(intakePower);



    }
}
