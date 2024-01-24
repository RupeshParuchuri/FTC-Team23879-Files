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
import com.qualcomm.robotcore.util.Range;

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
    private double powerLimit=0.5;

    public int getTargetPosition() {
        return targetPosition;
    }

    public void setTargetPosition(int targetPosition) {
        this.targetPosition = targetPosition;
    }
    public void setTargetPosition (int targetPosition, double powerLimit) {
        this.targetPosition = targetPosition;
        this.powerLimit = powerLimit;
    }
    public void setPower(double power)
    {
        if (power > 0.0)
        {
            // Move elevator towards max position with specified power.
            setTargetPosition(-400, power);
        }
        else if (power < 0.0)
        {
            // Move elevator towards min position with specified power.
            setTargetPosition(-100, power);
        }
        else
        {
            // Hold elevator position without power limit.
            setTargetPosition(targetPosition, 1.0);
        }
    }
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

    // This is called in automomous.
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

            armMotor.setPower(power);

            telemetry.update();
            return power;
    }

    //teleop...limiting the power
    public double moveTo() {

        pidController.setPID(kp,ki,kd);
        int armPos = armMotor.getCurrentPosition();

        //double targetPositionTicks = ((targetDeg-zeroOffset)*ticksPerDegree)/armPos;
        double pid = pidController.calculate(armPos, targetPosition);
        telemetry.addData("PID", pid);
        // double feedforward = Math.cos(Math.toRadians(targetMotorPosition / ticksPerDegree)) * kf;
        double angel = armPos / ticksPerDegree + zeroOffset;
        double feedforward = Math.sin(Math.toRadians(angel)) * kf;
        telemetry.addData("feed forward", feedforward);
        telemetry.addData("arm position:",armPos);
        telemetry.addData("angle",angel);
        double armPositionInDeg = armPos / ticksPerDegree + zeroOffset;
        double power = pid + feedforward;
        //power = Range.clip(power, -powerLimit, powerLimit);
        telemetry.addData("power", power);
        telemetry.addData("targetExtendPosition", targetPosition);

        armMotor.setPower(power);

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
            moveTo(targetPosition);

            return true;

        }
    }


    public Action extendToDropAtBoard() {
        return new ExtendToDropAtSpikeAction();
    }
}
