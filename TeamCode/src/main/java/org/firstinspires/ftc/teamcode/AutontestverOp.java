package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/*
NOTES
THIS VERSION IS FOR TESTING
NOT FOR COMP USE
 */
@Autonomous(name = "Autontest", group = "Iterative Opmode")
public class AutontestverOp extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(hardwareMap, telemetry);
        RevBlinkinLedDriver.BlinkinPattern pattern;



        double dist1;
        double dist2;
        double r, g, b;
        int pos;

        waitForStart();
        //farside blue
        //shift to measure distances
        while (opModeIsActive()) {


            double[] CurrentCoords = new double[]{0, 0, 0,
                    robot.LF.getCurrentPosition(), robot.RF.getCurrentPosition(),
                    robot.LB.getCurrentPosition()};
            robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
            //test distance sensor measuring
            //doesn't move to align sensors

            dist1 = robot.dSensor0.getDistance(DistanceUnit.INCH);
            dist2 = robot.dSensor1.getDistance(DistanceUnit.INCH);
            sleep(500);


            if (robot.DSCheck(dist1, dist2) == 2) {
                pos = 2;
                robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_RED);
                sleep(10000);
            } else if (robot.DSCheck(dist1, dist2) == 3) {
                pos = 3;
                robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_BLUE);
                sleep(10000);
            } else {
                pos = 1;
                robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_GOLD);
                sleep(10000);
            }

            //test color sensor
            r = robot.cSensor.red();
            g = robot.cSensor.green();
            b = robot.cSensor.blue();
            robot.colorCheck(r, g, b);
            telemetry.addData("coords", CurrentCoords);

            telemetry.addData("position", pos);

            //robot.MoveDirection(0, 0, .5, 1); // Direction (-180 ≤ angle ≤ 180), Turn (-1 ≤ turn ≤ 1), Throttle (1 is max speed possible), time is in seconds
            //robot.MoveDirection(0, 90, 0.5, 1);
            //robot.MoveDirection(0, 0, 0.5, 2);

            telemetry.addLine("End of Autonomous");
            telemetry.update();

        }
    }
}
