package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * CR舵机调速与极限测试程序
 * 操作系统：
 * - Dpad 上: 增加正向速度 (+0.05)
 * - Dpad 下: 增加反向速度 (-0.05)
 * - Dpad 左右: 立即停止 (0.5)
 * - A 键 / B 键: 微调步长切换
 */
@TeleOp(name = "CR舵机转速测试", group = "Test")
public class ServoTestOpMode extends LinearOpMode {

    private Servo feedServo;
    private double currentPowerPos = 0.5; // 0.5 为停止点
    private double stepSize = 0.05;       // 调节步长

    // 边缘检测变量
    private boolean dpadUpPrev = false;
    private boolean dpadDownPrev = false;
    private boolean dpadLeftPrev = false;
    private boolean dpadRightPrev = false;

    @Override
    public void runOpMode() {
        // 尝试寻找 Control Hub 0 口的舵机
        feedServo = null;
        for (Servo servo : hardwareMap.servo) {
            if (servo.getPortNumber() == 0) {
                feedServo = servo;
                break;
            }
        }

        telemetry.addData("0口舵机状态", feedServo == null ? "【未找到！】请检查配置" : "已连接");
        telemetry.addLine("======================================");
        telemetry.addLine("操作说明：");
        telemetry.addLine("Dpad 上：正向加速");
        telemetry.addLine("Dpad 下：反向加速");
        telemetry.addLine("Dpad 左右：紧急停止 (归零至 0.5)");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            boolean up = gamepad1.dpad_up;
            boolean down = gamepad1.dpad_down;
            boolean left = gamepad1.dpad_left;
            boolean right = gamepad1.dpad_right;

            // Dpad 上：增加数值（向 1.0 靠拢）
            if (up && !dpadUpPrev) {
                currentPowerPos += stepSize;
            }
            dpadUpPrev = up;

            // Dpad 下：减小数值（向 0.0 靠拢）
            if (down && !dpadDownPrev) {
                currentPowerPos -= stepSize;
            }
            dpadDownPrev = down;

            // Dpad 左右：归零停止
            if ((left && !dpadLeftPrev) || (right && !dpadRightPrev)) {
                currentPowerPos = 0.5;
            }
            dpadLeftPrev = left;
            dpadRightPrev = right;

            // 限制在 0.0 ~ 1.0 之间
            currentPowerPos = Math.max(0.0, Math.min(1.0, currentPowerPos));

            // 写入舵机
            if (feedServo != null) {
                feedServo.setPosition(currentPowerPos);
            }

            // 实时遥测显示（Driver Station 查看）
            telemetry.addData("=== CR舵机实时测试 ===", "");
            telemetry.addData("当前位置信号 (0~1)", "%.2f (0.5为停)", currentPowerPos);
            double speedFromStop = Math.abs(currentPowerPos - 0.5);
            telemetry.addData("相对静止的力度", "%.2f (最大 0.5)", speedFromStop);
            telemetry.addData("旋转状态", currentPowerPos > 0.5 ? "正向转动 -->" : (currentPowerPos < 0.5 ? "<-- 反向转动" : "【已停止】"));
            telemetry.addLine("--------------------------------------");
            telemetry.addLine("提示：如果 0.0 或 1.0 时速度依然不够，说明触及电机物理极限，需考虑齿轮增速。");
            telemetry.update();
        }

        // 退出时归零
        if (feedServo != null) {
            feedServo.setPosition(0.5);
        }
    }
}