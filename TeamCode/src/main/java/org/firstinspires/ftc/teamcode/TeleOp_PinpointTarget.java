package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "TeleOp_PinpointTarget", group = "Experiment")
public class TeleOp_PinpointTarget extends LinearOpMode {

    private Follower follower;
    private final ElapsedTime loopTimer = new ElapsedTime();

    // 摇杆死区
    private static final double DEADZONE = 0.05;
    // 虚拟目标移动的最大速度（单位：英寸/秒，可根据手感调节）
    private static final double MAX_TARGET_SPEED_IN_S = 40.0;
    // 虚拟目标转向的最大角速度（单位：弧度/秒）
    private static final double MAX_TARGET_TURN_SPEED = Math.PI;

    @Override
    public void runOpMode() {
        telemetry.addLine("==== 正在初始化 PedroPathing Follower ====");
        telemetry.update();

        try {
            // 直接调用 Constants.create 初始化 PedroPathing Follower
            follower = Constants.create(hardwareMap);
        } catch (Exception e) {
            telemetry.addLine("初始化失败: " + e.getMessage());
            telemetry.update();
            while (opModeInInit()) { sleep(50); }
            return;
        }

        // 归零位姿
        follower.setPose(Pose.zero());

        telemetry.addLine("==== PedroPathing 虚拟目标追踪就绪 ====");
        telemetry.update();

        waitForStart();

        follower.setPose(Pose.zero());
        loopTimer.reset();

        // 初始化 Pedro 目标 Pose (X, Y 单位为英寸，Heading 单位为弧度)
        Pose targetPose = Pose.zero();

        while (opModeIsActive()) {
            double dt = loopTimer.seconds();
            loopTimer.reset();

            // 1. 持续更新 Follower
            follower.update();

            // 获取当前机器人在场地中的实际位姿
            Pose currentPose = follower.pose();

            // 2. 读取手柄输入并经过死区处理（已将 left_stick_x 取反）
            double rawAxial = curve(-gamepad1.left_stick_y);   // 前后移动
            double rawLateral = curve(-gamepad1.left_stick_x); // 左右平移（已取反）
            double rawYaw = curve(-gamepad1.right_stick_x);   // 旋转

            // 3. 将摇杆指令转化为“虚拟目标点”的增量更新
            if (Math.hypot(rawAxial, rawLateral) > 0.01) {
                double dXBody = rawAxial * MAX_TARGET_SPEED_IN_S * dt;
                double dYBody = rawLateral * MAX_TARGET_SPEED_IN_S * dt;

                double currentHeading = currentPose.heading();
                double cosH = Math.cos(currentHeading);
                double sinH = Math.sin(currentHeading);

                // 累加到虚拟目标坐标上
                double newTargetX = targetPose.x() + (cosH * dXBody - sinH * dYBody);
                double newTargetY = targetPose.y() + (sinH * dXBody + cosH * dYBody);
                targetPose = new Pose(newTargetX, newTargetY, targetPose.heading());
            }

            // 处理旋转目标增量
            if (Math.abs(rawYaw) > 0.01) {
                double newTargetHeading = targetPose.heading() + (rawYaw * MAX_TARGET_TURN_SPEED * dt);
                targetPose = new Pose(targetPose.x(), targetPose.y(), newTargetHeading);
            }

            // 4. 调用 PedroPathing 的 hold 方法进行高精度闭环追踪
            follower.hold(targetPose);

            // 5. 实时遥测输出
            telemetry.addData("【当前位姿 (Pose)】", "X: %.1f in, Y: %.1f in, Heading: %.1f°",
                    currentPose.x(), currentPose.y(), Math.toDegrees(currentPose.heading()));
            telemetry.addData("【目标位姿 (Target)】", "X: %.1f in, Y: %.1f in, Heading: %.1f°",
                    targetPose.x(), targetPose.y(), Math.toDegrees(targetPose.heading()));
            telemetry.addData("【跟随状态】", "是否忙碌: %s", follower.isBusy() ? "是" : "稳定保持中 (Holding)");
            telemetry.update();
        }
    }

    /**
     * 带死区和线性缩放的摇杆曲线函数
     */
    private double curve(double raw) {
        double magnitude = Math.abs(raw);
        if (magnitude <= DEADZONE) return 0.0;
        double scaled = (magnitude - DEADZONE) / (1.0 - DEADZONE);
        return Math.copySign(Math.min(scaled, 1.0), raw);
    }
}