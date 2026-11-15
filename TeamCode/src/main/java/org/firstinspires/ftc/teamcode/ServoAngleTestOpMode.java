package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Map;

/**
 * 前部双舵机（1口上方 + 2口下方）联动角度测试程序
 * 保持原有映射关系：
 * - 上方角度 = 下方角度 - 33.0°
 * - 上方正向 (UPPER_REVERSED = false)，下方反向 (LOWER_REVERSED = true)
 *
 * 操作说明：
 * - Dpad 上 / 下: 下方基准角度 ±1.0°（上方联动跟随）
 * - Dpad 左: 快速跳转至 45°（中置挡球参考位）
 * - Dpad 右: 快速跳转至 75°（抬起参考位）
 */
@TeleOp(name = "前部双舵机联动测试", group = "Test")
public class ServoAngleTestOpMode extends LinearOpMode {

    // 与 RobotHardwareV0 保持完全一致的映射参数
    private static final int UPPER_SERVO_PORT = 1;  // Control Hub 1口：上方舵机
    private static final int LOWER_SERVO_PORT = 2;  // Control Hub 2口：下方舵机

    private static final double FRONT_RANGE_DEG = 180.0;
    private static final boolean UPPER_REVERSED = false;
    private static final boolean LOWER_REVERSED = true;
    private static final double UPPER_OFFSET_DEG = 33.0;

    private Servo frontUpper;
    private Servo frontLower;

    // 基准角度（以下方舵机为准，初始设为 45° 安全中间位，避免开机打到极限）
    private double lowerAngleDeg = 45.0;

    // 边缘检测变量
    private boolean dpadUpPrev = false;
    private boolean dpadDownPrev = false;
    private boolean dpadLeftPrev = false;
    private boolean dpadRightPrev = false;

    @Override
    public void runOpMode() {
        frontUpper = findServoByPort(UPPER_SERVO_PORT);
        frontLower = findServoByPort(LOWER_SERVO_PORT);

        telemetry.addData("1口上方舵机", frontUpper == null ? "【未找到】" : "OK");
        telemetry.addData("2口下方舵机", frontLower == null ? "【未找到】" : "OK");
        telemetry.addLine("======================================");
        telemetry.addLine("Dpad 上/下：基准角度 ±1.0°");
        telemetry.addLine("Dpad 左：跳转 45° | Dpad 右：跳转 75°");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            boolean up = gamepad1.dpad_up;
            boolean down = gamepad1.dpad_down;
            boolean left = gamepad1.dpad_left;
            boolean right = gamepad1.dpad_right;

            if (up && !dpadUpPrev) {
                lowerAngleDeg += 1.0;
            }
            dpadUpPrev = up;

            if (down && !dpadDownPrev) {
                lowerAngleDeg -= 1.0;
            }
            dpadDownPrev = down;

            if (left && !dpadLeftPrev) {
                lowerAngleDeg = 45.0;
            }
            dpadLeftPrev = left;

            if (right && !dpadRightPrev) {
                lowerAngleDeg = 75.0;
            }
            dpadRightPrev = right;

            // 限制范围：保证下方在 0~180° 且上方 (lower - 33) 不小于 0°（防止超程堵转）
            lowerAngleDeg = Math.max(UPPER_OFFSET_DEG, Math.min(FRONT_RANGE_DEG, lowerAngleDeg));
            double upperAngleDeg = lowerAngleDeg - UPPER_OFFSET_DEG;

            // 计算并写入双舵机位置
            double lowerPos = angleToPos(lowerAngleDeg, FRONT_RANGE_DEG, LOWER_REVERSED);
            double upperPos = angleToPos(upperAngleDeg, FRONT_RANGE_DEG, UPPER_REVERSED);

            if (frontLower != null) {
                frontLower.setPosition(lowerPos);
            }
            if (frontUpper != null) {
                frontUpper.setPosition(upperPos);
            }

            // 实时遥测输出
            telemetry.addData("下方舵机 (2口)", "%.1f°  (pos: %.4f, %s)",
                    lowerAngleDeg, lowerPos, LOWER_REVERSED ? "反向" : "正向");
            telemetry.addData("上方舵机 (1口)", "%.1f°  (pos: %.4f, %s)",
                    upperAngleDeg, upperPos, UPPER_REVERSED ? "反向" : "正向");
            telemetry.addLine("--------------------------------------");
            telemetry.addLine("调试完成后，将下方舵机度数填入 RobotHardwareV0 的 LOWER_DEG_* 常量即可。");
            telemetry.update();
        }
    }

    private Servo findServoByPort(int port) {
        for (Map.Entry<String, Servo> entry : hardwareMap.servo.entrySet()) {
            if (entry.getValue().getPortNumber() == port) {
                return entry.getValue();
            }
        }
        return null;
    }

    private static double angleToPos(double angleDeg, double rangeDeg, boolean reversed) {
        double p = angleDeg / rangeDeg;
        double clamped = Math.max(0.0, Math.min(1.0, p));
        return reversed ? 1.0 - clamped : clamped;
    }
}