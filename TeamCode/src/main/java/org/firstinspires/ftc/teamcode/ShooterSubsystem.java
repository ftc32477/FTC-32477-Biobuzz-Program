package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * 发射机构独立子系统 (同轴双电机版本：单 PIDF 计算，双电机同步输出)
 * 适用于 5203 1:1 同轴刚性连接电机 (28 ticks/rev)
 */
public class ShooterSubsystem {

    // ===================== 硬件与物理参数 =====================
    public static final double TICKS_PER_REV = 28.0;
    public static final double MAX_RPM = 6000.0;
    public static final double DEFAULT_TARGET_RPM = 4500.0;
    public static final double RPM_TOLERANCE = 100.0;
    public static final long SETTLE_TIME_MS = 250L;

    // ===================== PIDF 调参常量 =====================
    public static double kP = 0.0006;
    public static double kI = 0.00015;
    public static double kD = 0.0;
    public static double kF = 0.00024;

    private final DcMotor shooterL;
    private final DcMotor shooterR;
    private final boolean rightReversed;

    // 只需要一个 PIDF 控制器来统筹同轴转速
    private final VelocityPIDF pid = new VelocityPIDF();
    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime settleTimer = new ElapsedTime();

    private boolean enabled = false;
    private double targetRpm = DEFAULT_TARGET_RPM;
    private boolean isSettling = false;
    private double lastUpdateSec = 0.0;

    // 实时状态缓存
    private double currentRpm = 0.0;
    private double outputPower = 0.0;

    public ShooterSubsystem(DcMotor shooterL, DcMotor shooterR, boolean rightReversed) {
        this.shooterL = shooterL;
        this.shooterR = shooterR;
        this.rightReversed = rightReversed;
        loopTimer.reset();
    }

    public void setEnabled(boolean enabled) {
        if (this.enabled != enabled) {
            this.enabled = enabled;
            pid.reset();
            isSettling = false;
            lastUpdateSec = loopTimer.seconds();
        }
    }

    public boolean isEnabled() {
        return enabled;
    }

    public void setTargetRpm(double rpm) {
        this.targetRpm = Math.max(0.0, Math.min(MAX_RPM, rpm));
    }

    public void adjustTargetRpm(double deltaRpm) {
        setTargetRpm(this.targetRpm + deltaRpm);
    }

    public void resetTargetRpm() {
        setTargetRpm(DEFAULT_TARGET_RPM);
    }

    public double getTargetRpm() {
        return targetRpm;
    }

    /** 主循环周期性调用：单 PIDF 计算，统一分配给同轴左右电机 */
    public void update() {
        double nowSec = loopTimer.seconds();
        double dt = nowSec - lastUpdateSec;
        lastUpdateSec = nowSec;

        if (dt <= 1e-4 || dt > 0.25) {
            dt = 0.02;
        }

        // 以左电机（或左右平均）作为反馈源
        double velTicksL = getMotorVelocityTicks(shooterL);
        double velTicksR = getMotorVelocityTicks(shooterR);
        double avgVelTicks = (velTicksL + velTicksR) / 2.0; // 取双电机平均转速作为反馈，更平稳
        currentRpm = ticksToRpm(avgVelTicks);

        if (!enabled || shooterL == null || shooterR == null || targetRpm <= 0.0) {
            outputPower = 0.0;
            isSettling = false;
            setMotorPower(0.0);
            return;
        }

        double targetTicks = rpmToTicks(targetRpm);
        // 单个 PIDF 统一算出当前应该给的功率
        outputPower = pid.calculate(targetTicks, avgVelTicks, dt);

        // 将同一个计算出的功率同步下发给同轴双电机，彻底消除“内耗打架”
        setMotorPower(outputPower);

        // 转速门判定
        boolean withinTolerance = Math.abs(currentRpm - targetRpm) <= RPM_TOLERANCE;

        if (withinTolerance) {
            if (!isSettling) {
                isSettling = true;
                settleTimer.reset();
            }
        } else {
            isSettling = false;
        }
    }

    public boolean isReadyToFire() {
        return enabled
                && shooterL != null
                && shooterR != null
                && isSettling
                && settleTimer.milliseconds() >= SETTLE_TIME_MS;
    }

    public double getCurrentRpm() { return currentRpm; }
    public double getOutputPower() { return outputPower; }

    private void setMotorPower(double power) {
        if (shooterL != null) {
            shooterL.setPower(power);
        }
        if (shooterR != null) {
            shooterR.setPower(rightReversed ? -power : power);
        }
    }

    private static double getMotorVelocityTicks(DcMotor motor) {
        if (motor instanceof DcMotorEx) {
            return Math.abs(((DcMotorEx) motor).getVelocity());
        }
        return 0.0;
    }

    private static double rpmToTicks(double rpm) {
        return rpm * TICKS_PER_REV / 60.0;
    }

    private static double ticksToRpm(double ticksPerSec) {
        return ticksPerSec * 60.0 / TICKS_PER_REV;
    }

    private static class VelocityPIDF {
        private static final double I_MAX_OUTPUT = 0.3;
        private double integral = 0.0;
        private double lastError = 0.0;

        double calculate(double target, double measured, double dt) {
            double error = target - measured;
            integral += error * dt;
            if (kI != 0.0) {
                double maxI = I_MAX_OUTPUT / Math.abs(kI);
                integral = Math.max(-maxI, Math.min(maxI, integral));
            }
            double derivative = (error - lastError) / dt;
            lastError = error;

            double out = (kF * target) + (kP * error) + (kI * integral) + (kD * derivative);
            return Math.max(-1.0, Math.min(1.0, out));
        }

        void reset() {
            integral = 0.0;
            lastError = 0.0;
        }
    }
}