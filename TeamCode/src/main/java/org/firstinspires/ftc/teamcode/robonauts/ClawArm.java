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
    private final double zeroOffset = 60;

    private PIDController pidController=null;
    public static  double kp=0.0004;//0.77;
    public static  double ki=0.00001;//0.003;
    public static  double kd=0.0001;//0.0001;

    public static double kf=0.0085;//0.03;

    //public static int targetDeg=0;
    private final double ticksPerDegree=2786.2 / 360;
    private Telemetry telemetry;
    private double powerLimit=0.5;

    public int getTargetPosition() {
        return targetPosition;
    }

    public Context getRobotState() {
        return robotState;
    }

    public void setTargetPosition(int targetPosition) {
        this.targetPosition = targetPosition;
    }
    public void setTargetPosition (int targetPosition, double powerLimit) {
        this.targetPosition = targetPosition;
        this.powerLimit = powerLimit;
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
            //power = Range.clip(power, -0.7, 0.7);

        telemetry.addData("power", power);
            telemetry.addData("targetExtendPosition", targetExtendPosition);

            armMotor.setPower(power);
             this.robotState.setArmPosition(armPos);

            telemetry.update();
            return power;
    }

    //teleop...limiting the power
    public double moveTo() {

        pidController.setPID(kp,ki,kd);
        int armPos =  armMotor.getCurrentPosition();
        //double targetPositionTicks = ((targetDeg-zeroOffset)*ticksPerDegree)/armPos;
        double pid = pidController.calculate(armPos, targetPosition);
        // double feedforward = Math.cos(Math.toRadians(targetMotorPosition / ticksPerDegree)) * kf;
        double angel = armPos / ticksPerDegree + zeroOffset;
        double feedforward = Math.sin(Math.toRadians(angel)) * kf;
        double armPositionInDeg = armPos/ticksPerDegree + zeroOffset;
        double power = pid + feedforward;
        armMotor.setPower(power);
       // power = Range.clip(power, -0.7, 0.7);
        telemetry.addData("power", power);
        telemetry.addData("targetExtendPosition", targetPosition);

        armMotor.setPower(power);
        this.robotState.setArmPosition(armPos);

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

    public void setRobotState() {
        this.robotState.setRobotState("ARMEXTENDED");
    }

    public void setPower(double armPower) {
        armPower = Range.clip(armPower, -0.042, 0.042);

        armMotor.setPower(armPower);
    }



    public class ExtendToDropAtSpikeAction implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                initialized = true;
            }
            telemetryPacket.put("Current state in CLAW ARM...", robotState.getRobotState() + System.currentTimeMillis());

                moveTo(targetPosition);
                if (Math.abs(robotState.getArmPosition()) > 250) {
                    setRobotState();
                    telemetryPacket.put("Context set in ClawArm...", robotState.getRobotState() + System.currentTimeMillis());
                    return false;
                } else {
                    return true;
                }

            }


    }


    public Action extendToDropAtBoard() {

        return new ExtendToDropAtSpikeAction();
    }
}
