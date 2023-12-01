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
            Pose2d poseEstimate = drive.driveseq(0,0,3,0,0, startPose);
            drive.DSCheck();
            if (drive.DSCheck() == 2) {
                poseEstimate = drive.driveseq(25,0,0,3,0, poseEstimate);

                //drop off pixel
                poseEstimate = drive.driveseq(0,10,0,0,0, poseEstimate);

                poseEstimate = drive.driveseq(0,0,20,0,0, poseEstimate);

                // line up parking path
                poseEstimate = drive.driveseq(12,0,0,0,0, poseEstimate);
                poseEstimate = drive.driveseq(12,0,25,0,-90, poseEstimate);

                //park
                poseEstimate = drive.driveseq(120,0,0,0,0, poseEstimate);




            } else if (drive.DSCheck() == 3) {
                poseEstimate = drive.driveseq(18,0,0,0,0, poseEstimate);

                //drop off pixel
                poseEstimate = drive.driveseq(0,10,0,0,0, poseEstimate);

                poseEstimate = drive.driveseq(0,0,20,0,0, poseEstimate);

                // line up parking path
                poseEstimate = drive.driveseq(19,0,0,0,0, poseEstimate);
                poseEstimate = drive.driveseq(12,0,25,0,-90, poseEstimate);

                //park
                poseEstimate = drive.driveseq(120,0,0,0,0, poseEstimate);


            } else {


            }

        }
    }

}
