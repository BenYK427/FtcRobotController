package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.mechanisms.Flywheel;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.MechanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Turret;

@TeleOp
public class BensAmazingTest extends OpMode{

    MechanumDrive drive = new MechanumDrive();
    double forward, strafe, turn;
    boolean adjust;

    Intake intake = new Intake();
    double intakePower;

    Flywheel flywheel = new Flywheel();
    double distance;
    boolean autoFlywheel;

    //Turret turret = new Turret();
    double turretPower;
    double turretPos;
    //private DcMotor turretMotor;
    GoBildaPinpointDriver odo;

    private Limelight3A limelight;


    private DcMotor turretMotor;
    double i;
    double d;
    double error;
    double lastError;
    double PID;
    double txpmo;
    double targetPos = 0;


    public void init(){
        drive.init(hardwareMap);
        intake.init(hardwareMap);
        flywheel.init(hardwareMap);
        //turret.init(hardwareMap);

        turretMotor = hardwareMap.get(DcMotor.class, "turretMotor");

        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        odo.resetPosAndIMU();
        Pose2D startingPosition = new Pose2D(DistanceUnit.MM, 100, 100, AngleUnit.RADIANS, 0);
        odo.setPosition(startingPosition);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);

        telemetry.addLine("init complete");

    }

    public void start(){
        limelight.start();
    }

    @Override
    public void loop() {
        //-----------------------Odometry---------------------------
        double heading = odo.getHeading(UnnormalizedAngleUnit.RADIANS);
        double headingNorm = odo.getHeading(AngleUnit.RADIANS);

        double x = odo.getPosX(DistanceUnit.MM);

        //----------------------------Limelight-------------------------
        limelight.updateRobotOrientation(headingNorm);
        LLResult llResult = limelight.getLatestResult();

        Pose3D botPose = llResult.getBotpose();
        double tx = llResult.getTx();




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



        if(gamepad2.dpadUpWasPressed()){
            targetPos += 100;
        }
        if(gamepad2.dpadDownWasPressed()){
            targetPos -= 100;
        }

        targetPos = heading*800/3.14159;

        error = turretMotor.getCurrentPosition() - targetPos;
        i = i + error;
        if(Math.abs(error) > 10){
            i = 0;
        }
        d = error - lastError;
        lastError = error;
        //p:0.45 i:0.05 d:0.5

        PID = error * 1 + i * 0.05 + d * 0.05;
        PID = PID*0.007;

        if(PID > 0.6){
            PID = 0.6;
        }

        if(PID < -0.6){
            PID = -0.6;
        }

        if(Math.abs(error) < 5){
            PID = 0;
        }

//        if(PID > 0.4){
//            PID = 0.4;
//        }
//
//        if(PID < -0.4){
//            PID = -0.4;
//        }

        if(gamepad2.b){
            PID = 0;
        }


        turretMotor.setPower(-PID);


        //turretMotor.setPower(PID + turretPower);


        //turret.turret(turretPower);
        //----------------------Flywheel------------------------
        distance = 2000;

        if(gamepad2.rightBumperWasPressed()){
            autoFlywheel = !autoFlywheel;
        }

        flywheel.flywheel(distance, autoFlywheel);

        //--------------------Telemetry-------------------------

        telemetry.addData("heading", heading*800/3.14159);
        telemetry.addData("x", x);
        //telemetry.addData("status", autoFlywheel);
        telemetry.addData("tx", tx);
        telemetry.addData("turret Power", turretMotor.getPower());
        telemetry.addData("turretPos", turretMotor.getCurrentPosition());
        telemetry.update();

        odo.update();
    }
}
