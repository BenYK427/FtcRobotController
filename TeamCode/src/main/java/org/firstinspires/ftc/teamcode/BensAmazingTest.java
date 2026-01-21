package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.mechanisms.Flywheel;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.MechanumDrive;

@TeleOp
public class BensAmazingTest extends OpMode{

    MechanumDrive drive = new MechanumDrive();
    double forward, strafe, turn;
    boolean adjust;

    Intake intake = new Intake();
    double intakePower;

//    Flywheel flywheel = new Flywheel();
//    double distance;
//    boolean autoFlywheel;

    GoBildaPinpointDriver odo;


    public void init(){
        drive.init(hardwareMap);
        intake.init(hardwareMap);
        //flywheel.init(hardwareMap);

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        odo.resetPosAndIMU();
        Pose2D startingPosition = new Pose2D(DistanceUnit.MM, 100, 100, AngleUnit.RADIANS, 0);
        odo.setPosition(startingPosition);

        telemetry.addLine("init complete");

    }

    @Override
    public void loop() {
        //-----------------------Odometry---------------------------
        double heading = odo.getHeading(UnnormalizedAngleUnit.RADIANS);

        double x = odo.getPosX(DistanceUnit.MM);

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


        //-----------------------Intake-------------------------
        intakePower = gamepad2.left_stick_y;

        intake.intake(intakePower);

        //-----------------------Turret-------------------------

        //----------------------Flywheel------------------------
//        distance = 1500;
//
//        if(gamepad2.rightBumperWasPressed()){
//            autoFlywheel = !autoFlywheel;
//        }
//
//        flywheel.flywheel(distance, autoFlywheel);

        //--------------------Telemetry-------------------------

        telemetry.addData("heading", heading);
        telemetry.addData("x", x);
        //telemetry.addData("status", autoFlywheel);
        telemetry.update();

        odo.update();
    }
}
