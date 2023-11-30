package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous(name = "Auton", group = "Iterative Opmode")
public class AutonOp extends LinearOpMode {
    public void runOpMode() throws InterruptedException {

        RobotHardware robot = new RobotHardware(hardwareMap, telemetry);
        double[] CurrentCoords = new double[]{0, 0, 0,
                robot.LF.getCurrentPosition(), robot.RF.getCurrentPosition(),
                robot.LB.getCurrentPosition()};
        RevBlinkinLedDriver.BlinkinPattern pattern;

        double dist1;
        double dist2;
        double r, g, b;
        int pos;

        waitForStart();
        while (opModeIsActive() ){
            //farside blue
            //shift to measure distances
            CurrentCoords = robot.driveToTile(CurrentCoords, new double[]{0.25, 0, 0}, 1);
            dist1 = robot.dSensor0.getDistance(DistanceUnit.INCH);
            dist2 = robot.dSensor1.getDistance(DistanceUnit.INCH);
            sleep(500);
        /*
        //conditionals for each position
        if (robot.DSCheck(dist1,dist2) ==2){
            pos = 2;
            CurrentCoords = robot.driveToTile(CurrentCoords, new double []{0, 0, 0}, 1);
            sleep(20);
            CurrentCoords = robot.driveToTile(CurrentCoords, new double []{0, 0, 0}, 1);
        } else if (robot.DSCheck(dist1,dist2)==3) {
            pos =3;
            CurrentCoords = robot.driveToTile(CurrentCoords, new double []{0, 0, 0}, 1);
            sleep(20);
            CurrentCoords = robot.driveToTile(CurrentCoords, new double []{0, 0, 0}, 1);
        }else {
            pos =1;
            CurrentCoords = robot.driveToTile(CurrentCoords, new double []{0, 0, 0}, 1);
            sleep(20);
            //turn to get behind pole in front of spike mark
            robot.turnDegrees(CurrentCoords, -90);
            CurrentCoords = robot.driveToTile(CurrentCoords, new double []{4.25, 2.38, 0}, 1);
        }


    */

            //telemetry.addData("position", pos);

            //robot.MoveDirection(0, 0, .5, 1); // Direction (-180 ≤ angle ≤ 180), Turn (-1 ≤ turn ≤ 1), Throttle (1 is max speed possible), time is in seconds
            //robot.MoveDirection(0, 90, 0.5, 1);
            //robot.MoveDirection(0, 0, 0.5, 2);

            telemetry.addLine("End of Autonomous");
            telemetry.update();

        }
    }
}
