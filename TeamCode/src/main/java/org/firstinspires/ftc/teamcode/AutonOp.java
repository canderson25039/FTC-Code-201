package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Autonomous(name = "Autonbasic", group = "Iterative Opmode")
public class AutonOp extends LinearOpMode {
    public void runOpMode() throws InterruptedException {


        RobotHardware robot = new RobotHardware(hardwareMap, telemetry);
        RevBlinkinLedDriver.BlinkinPattern pattern;

        double dist1;
        double dist2;
        double r, g, b;
        int pos;

        waitForStart();
        while (opModeIsActive() ){
            dist1 = robot.dSensor0.getDistance(DistanceUnit.INCH);
            dist2 = robot.dSensor1.getDistance(DistanceUnit.INCH);



            if (robot.DSCheck(dist1, dist2) == 2) {
                pos = 2;
                robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_RED);

            } else if (robot.DSCheck(dist1, dist2) == 3) {
                pos = 3;
                robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_BLUE);

            } else {
                pos = 1;
                robot.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_GOLD);

            }
            telemetry.addData("dist1", dist1);
            telemetry.addData("dist2", dist2);
            telemetry.addLine("End of Autonomous");
            telemetry.update();



        }
    }
}
