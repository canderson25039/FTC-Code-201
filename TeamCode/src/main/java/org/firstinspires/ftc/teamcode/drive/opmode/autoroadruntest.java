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

public class autoroadruntest extends LinearOpMode{

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
                    .strafeRight(3.25*multiply)
                    .forward(30)

                    .build();

            drive.followTrajectorySequence(trajSeq);
            Pose2d poseEstimate = drive.getPoseEstimate();
            sleep(5000);
            drive.DSCheck();
            if (drive.DSCheck() == 2) {

                TrajectorySequence case2traj1 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .strafeLeft(2.5*multiply)
                        .forward(13)
                        .build();
                drive.followTrajectorySequence(case2traj1);


                //drop pixel off
                case2traj1 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .back(20)
                        .build();
                drive.followTrajectorySequence(case2traj1);

                telemetry.addData("pos mid", null);
                case2traj1 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .strafeRight(4.5*multiply)
                        .forward(40)
                        .turn(Math.toRadians(-120))
                        .forward(100)
                        .build();
                drive.followTrajectorySequence(case2traj1);



                 break;



            } else if (drive.DSCheck() == 3) {
                TrajectorySequence case2traj1 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .strafeRight(3*multiply)
                        .forward(10)
                        .build();
                drive.followTrajectorySequence(case2traj1);


                //drop pixel off
                case2traj1 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .back(10)
                        .build();
                drive.followTrajectorySequence(case2traj1);

                telemetry.addData("pos right ", null);



                break;

            } else {
                TrajectorySequence case2traj1 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .strafeLeft(2.5*multiply)

                        .build();
                drive.followTrajectorySequence(case2traj1);
                poseEstimate = drive.getPoseEstimate();

                case2traj1 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .turn(Math.toRadians(-100))
                        .forward(10)
                        .build();
                drive.followTrajectorySequence(case2traj1);
                poseEstimate = drive.getPoseEstimate();

                //drop pixel off
                case2traj1 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .back(10)
                        .build();
                drive.followTrajectorySequence(case2traj1);
                poseEstimate = drive.getPoseEstimate();

                telemetry.addData("pos left", null);



                break;

            }

        }
    }

}
