package org.firstinspires.ftc.teamcode.robonauts;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LinearSlides {
    DcMotorEx leftMotor = null;
    DcMotorEx rightMotor = null;

    HardwareMap hardwareMap;

    Telemetry telemetry;

    private PIDController pidController=null;
    public static  double kp=0.0012;//0.77;
    public static  double ki=0;//0.003;
    public static  double kd=0;//0.004;

    public static double kf=-0.0003;//0.03;

    private final double ticksPerDegree=537.7 / 360;

    private int targetPosition;

    private String pixelState;

    private ClawArm clawArm;

    private  ClawMain clawMain;


    public LinearSlides(HardwareMap hardwareMap, Telemetry telemetry, int targetPosition, String pixelState) {
        this(hardwareMap, telemetry, targetPosition, pixelState, null, null);
    }

    public LinearSlides(HardwareMap hardwareMap, Telemetry telemetry, int targetPosition,
                        String pixelState, ClawArm clawArm, ClawMain clawMain) {

        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.targetPosition = targetPosition;
        this.pixelState = pixelState;

        pidController = new PIDController(kp, ki,kd);
        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        this.clawArm = clawArm;
        this.clawMain = clawMain;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        pidController = new PIDController(kp, ki,kd);

        leftMotor = hardwareMap.get(DcMotorEx.class, "par0");
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        rightMotor = hardwareMap.get(DcMotorEx.class, "par1");


        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public double moveTo(double targetMotorPosition) {
        int leftArmPos =  leftMotor.getCurrentPosition();
        //double targetPositionTicks = ((targetDeg-zeroOffset)*ticksPerDegree)/armPos;
        double leftPid = pidController.calculate(leftArmPos, targetMotorPosition);
        // double feedforward = Math.cos(Math.toRadians(targetMotorPosition / ticksPerDegree)) * kf;
        double feedforward = kf;
        double leftpower = leftPid + feedforward;

        int rightArmPos =  rightMotor.getCurrentPosition();
        double rightPid = pidController.calculate(rightArmPos, targetMotorPosition);

        double rightpower = rightPid + feedforward;

        leftMotor.setPower(rightpower);
        rightMotor.setPower(rightpower);
        telemetry.addData("Feed Forward : " , feedforward);
        telemetry.addData("Left Power : " , leftpower);
        telemetry.addData("right Power : " , rightpower);

        telemetry.addData("Left slide current position : " , leftArmPos);
        telemetry.addData("Right slide current position : ", rightMotor.getCurrentPosition());
        telemetry.addData("target position : ", targetMotorPosition);
        telemetry.update();
        return rightpower;
    }

    public class ExtendToDropAtSpikeAction implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            moveTo(targetPosition);

            if (pixelState.equalsIgnoreCase("CLAW_RELEASED")) {
                return false;
            } else {
                return true;
            }
        }
    }

    public class ExtendToDropAtBoard implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            moveTo(targetPosition);
            //Thread.sleep(100);
            //clawArm.extend();
            //clawMain.releaseClaw();
            return true;
            /*if (pixelState.equalsIgnoreCase("CLAW_RELEASED")) {
                return false;
            } else {
                return true;
            }*/
        }
    }

    public Action extendToDropAtSpike() {
        return new LinearSlides.ExtendToDropAtSpikeAction();
    }

    public Action extendToDropAtBoard() {
        return new LinearSlides.ExtendToDropAtBoard();
    }
}
