package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
@TeleOp
public class Odometry  extends OpMode {

    GoBildaPinpointDriver odo;

    @Override
    public void init() {
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.resetPosAndIMU();
        Pose2D startingPosition = new Pose2D(DistanceUnit.MM, 100, 100, AngleUnit.RADIANS, 0);
        odo.setPosition(startingPosition);
    }


//    public void heading() {
//        double heading = odo.getHeading(UnnormalizedAngleUnit.RADIANS);
//    }

    @Override
    public void loop() {
        double heading = odo.getHeading(UnnormalizedAngleUnit.RADIANS);

        telemetry.addData("heading", heading);

        odo.update();
    }
}
