package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import com.qualcomm.robotcore.hardware.DistanceSensor;
//import org.firstinspires.ftc.teamcode.BasicTeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
@Config
@Autonomous(group = "drive")

public class autoguess extends LinearOpMode{

    double multiply = 9;


    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, 0);

        drive.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested()) {

            TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(drive.getPoseEstimate())

                    .forward(32)

                    .build();

            drive.followTrajectorySequence(trajSeq);
            Pose2d poseEstimate = drive.getPoseEstimate();
            trajSeq = drive.trajectorySequenceBuilder(drive.getPoseEstimate())

                    .back(10)

                    .build();

            drive.followTrajectorySequence(trajSeq);
            poseEstimate = drive.getPoseEstimate();
            sleep(5000);
            drive.DSCheck();




                break;

            }

        }
    }


