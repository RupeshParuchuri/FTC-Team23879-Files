package org.firstinspires.ftc.teamcode.robonauts;

import android.drm.DrmStore;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ClawArm {
    DcMotorEx armMotor = null;
    private final double zeroOffset = 30;

    private PIDController pidController=null;
    public static  double kp=0.001;//0.77;
    public static  double ki=0.00001;//0.003;
    public static  double kd=0.00001;//0.0001;

    public static double kf=0.0001;//0.03;


    //public static int targetDeg=0;
    private final double ticksPerDegree=1425.1 / 360;
    private Telemetry telemetry;

    private int targetPosition;

    private Context robotState;

    public ClawArm(HardwareMap hardwareMap, Telemetry telemetry, int targetPosition, Context robotState) {
        armMotor = hardwareMap.get(DcMotorEx.class, "TestMotor");
        pidController = new PIDController(kp, ki,kd);
        armMotor.setDirection(DcMotor.Direction.REVERSE);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.telemetry=telemetry;
        this.targetPosition = targetPosition;
        this.robotState = robotState;
    }
    public void setPower(double power) {
        armMotor.setPower(power);
    }
    public double moveTo(int targetExtendPosition) {

            pidController.setPID(kp,ki,kd);
            int armPos = armMotor.getCurrentPosition();

            //double targetPositionTicks = ((targetDeg-zeroOffset)*ticksPerDegree)/armPos;
            double pid = pidController.calculate(armPos, targetExtendPosition);
            telemetry.addData("PID", pid);
            // double feedforward = Math.cos(Math.toRadians(targetMotorPosition / ticksPerDegree)) * kf;
            double angel = armPos / ticksPerDegree + zeroOffset;
            double feedforward = Math.sin(Math.toRadians(angel)) * kf;
            telemetry.addData("feed forward", feedforward);
            telemetry.addData("arm position:",armPos);
            telemetry.addData("angle",angel);
            double armPositionInDeg = armPos / ticksPerDegree + zeroOffset;
            double power = pid + feedforward;
            telemetry.addData("power", power);
            telemetry.addData("targetExtendPosition", targetExtendPosition);

            //armMotor.setTargetPosition(targetExtendPosition);
            setPower(power);

            telemetry.update();
            return power;
    }

    public void extend() {
        moveTo(-300);
    }

    public void release() {
        //moveTo(0);
    }

    public double getCurrentPosition() {
        return armMotor.getCurrentPosition();
    }

    public class ExtendToDropAtSpikeAction implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            moveTo(500);
            /*if (robotState.getRobotState().equalsIgnoreCase("CLAW_RELEASED")) {
                return false;
            } else {
                return true;
            }*/
            return true;

        }
    }

    public Action extendToDropAtSpike() {
        return new ExtendToDropAtSpikeAction();
    }

    public Action extendToDropAtBoard() {
        return new ExtendToDropAtSpikeAction();
    }
}
