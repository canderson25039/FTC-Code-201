package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous(name = "Auton", group = "Iterative Opmode")
public class AutonOp extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(hardwareMap, telemetry);
        double dist1;
        double dist2;
        int pos;
        waitForStart();
        dist1= robot.dSensor0.getDistance(DistanceUnit.INCH);
        dist2 = robot.dSensor1.getDistance(DistanceUnit.INCH);
        sleep(500);
        if (robot.DSCheck(dist1,dist2) ==1){
            pos = 1;

        } else if (robot.DSCheck(dist1,dist2)==2) {
            pos =2;

        }else {
            pos =3;

        }
        telemetry.addData("position", pos);
        robot.MoveDirection(0, 0, .5, 1); // Direction (-180 ≤ angle ≤ 180), Turn (-1 ≤ turn ≤ 1), Throttle (1 is max speed possible), time is in seconds
        robot.MoveDirection(0, 90, 0.5, 1);
        robot.MoveDirection(0, 0, 0.5, 2);

        telemetry.addLine("End of Autonomous");
        telemetry.update();

    }

}
