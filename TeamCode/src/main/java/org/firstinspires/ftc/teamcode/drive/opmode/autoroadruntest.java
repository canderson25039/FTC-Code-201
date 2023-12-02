package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
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

public class autoroadruntest extends LinearOpMode{




    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, 0);

        drive.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested()) {

            TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                    .strafeRight(3)
                    .build();
            drive.followTrajectorySequence(trajSeq);
            Pose2d poseEstimate = drive.getPoseEstimate();
                    drive.DSCheck();
            if (drive.DSCheck() == 2) {
                TrajectorySequence case2traj1 = drive.trajectorySequenceBuilder(poseEstimate)
                        .strafeLeft(3)
                        .forward(25)
                        .build();
                drive.followTrajectorySequence(case2traj1);
                poseEstimate = drive.getPoseEstimate();

                //drop pixel off
                case2traj1 = drive.trajectorySequenceBuilder(poseEstimate)
                        .back(10)
                        .build();
                drive.followTrajectorySequence(case2traj1);
                poseEstimate = drive.getPoseEstimate();

                //line up parking path
                case2traj1 = drive.trajectorySequenceBuilder(poseEstimate)
                        .strafeRight(20)
                        .build();
                drive.followTrajectorySequence(case2traj1);
                poseEstimate = drive.getPoseEstimate();

                case2traj1 = drive.trajectorySequenceBuilder(poseEstimate)
                        .forward(24)
                        .build();
                drive.followTrajectorySequence(case2traj1);
                poseEstimate = drive.getPoseEstimate();

                case2traj1 = drive.trajectorySequenceBuilder(poseEstimate)
                        .turn(Math.toRadians(-90))
                        .build();
                drive.followTrajectorySequence(case2traj1);
                poseEstimate = drive.getPoseEstimate();

                //drive to park
                case2traj1 = drive.trajectorySequenceBuilder(poseEstimate)
                        .forward(145)
                        .build();
                drive.followTrajectorySequence(case2traj1);
                poseEstimate = drive.getPoseEstimate();




            } else if (drive.DSCheck() == 3) {


            } else {


            }

        }
    }

}
