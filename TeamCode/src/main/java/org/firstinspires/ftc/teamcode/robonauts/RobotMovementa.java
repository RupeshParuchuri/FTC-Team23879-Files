package org.firstinspires.ftc.teamcode.robonauts;

import androidx.core.app.NotificationChannelCompat;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;


public class RobotMovementa {
    Telemetry telemetry = null;
    Pose2d beginPose = null;
    MecanumDrive drive = null;

    public RobotMovementa(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.beginPose = new Pose2d(0, 0, 0);
        this.drive = new MecanumDrive(hardwareMap, beginPose);

    }

    public RobotMovementa(HardwareMap hardwareMap, Telemetry telemetry, Pose2d beginPose) {
        this.telemetry = telemetry;
        this.beginPose = beginPose;
    }

    public MecanumDrive moveToSpike(int x, int y) {
        Pose2d pose = new Pose2d(24, 0, 0);

        Actions.runBlocking(drive.
                actionBuilder(pose).
                //lineToXConstantHeading(x));
                turn(Math.toRadians(30) )
                .build());
        return drive;
    }

    private NotificationChannelCompat.Builder turn(double toRadians) {
        return null;
    }

    public MecanumDrive rotate(int i) {
        return null;
    }

    public void moveToSpikeLeft(int i, int i1) {
    }
}
