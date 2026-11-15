package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

@TeleOp(name = "TeleOp_PinpointTarget", group = "Experiment")
public class TeleOp_PinpointTarget extends LinearOpMode {

    public static final String PINPOINT_NAME = "odo";

    public static final boolean REAR_NAMES_SWAPPED = RobotHardwareV0.REAR_NAMES_SWAPPED;
    public static final boolean SWAP_LATERAL_YAW = RobotHardwareV0.SWAP_LATERAL_YAW;
    public static final double AXIAL_SIGN = RobotHardwareV0.AXIAL_SIGN;
    public static final double LATERAL_SIGN = RobotHardwareV0.LATERAL_SIGN;
    public static final double YAW_SIGN = RobotHardwareV0.YAW_SIGN;

    public static final DcMotor.Direction LEFT_FRONT_DIRECTION = RobotHardwareV0.LEFT_FRONT_DIRECTION;
    public static final DcMotor.Direction RIGHT_FRONT_DIRECTION = RobotHardwareV0.RIGHT_FRONT_DIRECTION;
    public static final DcMotor.Direction LEFT_BACK_DIRECTION = RobotHardwareV0.LEFT_BACK_DIRECTION;
    public static final DcMotor.Direction RIGHT_BACK_DIRECTION = RobotHardwareV0.RIGHT_BACK_DIRECTION;
    public static final DcMotor.ZeroPowerBehavior CHASSIS_ZERO_POWER = RobotHardwareV0.CHASSIS_ZERO_POWER;

    public static final double X_POD_OFFSET_MM = -74.0;
    public static final double Y_POD_OFFSET_MM = -136.0;

    private static final double DEADZONE = 0.05;
    private static final double MAX_TARGET_SPEED_MM_S = 1000.0;
    private static final double MAX_CHASSIS_POWER = 0.5;
    private static final double POSITION_P_GAIN = 0.003;
    private static final double HEADING_KP = 1.2;

    public DcMotor frontLeft, frontRight, backLeft, backRight;
    private GoBildaPinpointDriver pinpoint;
    private final ElapsedTime loopTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        try {
            frontLeft = findMotor("左前 (LF/端口0)", "lf", "LF");
            frontRight = findMotor("右前 (RF/端口1)", "rf", "RF");
            if (REAR_NAMES_SWAPPED) {
                backLeft = findMotor("左后 (端口3)", "rb", "RB");
                backRight = findMotor("右后 (端口2)", "lb", "LB");
            } else {
                backLeft = findMotor("左后 (LB/端口3)", "lb", "LB");
                backRight = findMotor("右后 (RB/端口2)", "rb", "RB");
            }

            configureChassisMotor(frontLeft, LEFT_FRONT_DIRECTION);
            configureChassisMotor(frontRight, RIGHT_FRONT_DIRECTION);
            configureChassisMotor(backLeft, LEFT_BACK_DIRECTION);
            configureChassisMotor(backRight, RIGHT_BACK_DIRECTION);
        } catch (IllegalStateException e) {
            telemetry.addLine("底盘电机初始化失败：");
            telemetry.addLine(e.getMessage());
            telemetry.update();
            while (opModeInInit()) { sleep(50); }
            return;
        }

        try {
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, PINPOINT_NAME);
        } catch (RuntimeException e) {
            telemetry.addLine("找不到 Pinpoint：" + e.getMessage());
            telemetry.update();
            while (opModeInInit()) { sleep(50); }
            return;
        }

        pinpoint.setOffsets(X_POD_OFFSET_MM, Y_POD_OFFSET_MM, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );

        pinpoint.resetPosAndIMU();

        telemetry.addLine("==== 纯底盘 Pinpoint 虚拟目标追踪就绪 ====");
        telemetry.update();

        waitForStart();

        pinpoint.resetPosAndIMU();
        loopTimer.reset();

        double targetX = 0.0;
        double targetY = 0.0;
        double targetHeading = 0.0;

        while (opModeIsActive()) {
            double dt = loopTimer.seconds();
            loopTimer.reset();

            pinpoint.update();

            Pose2D currentPose = pinpoint.getPosition();
            double currentX = currentPose.getX(DistanceUnit.MM);
            double currentY = currentPose.getY(DistanceUnit.MM);
            double currentHeading = currentPose.getHeading(AngleUnit.RADIANS);

            double rawAxial = curve(-gamepad1.left_stick_y);
            double rawLateral = curve(gamepad1.left_stick_x);
            double rawYaw = curve(gamepad1.right_stick_x);

            if (Math.hypot(rawAxial, rawLateral) > 0.01) {
                double dXBody = rawAxial * MAX_TARGET_SPEED_MM_S * dt;
                double dYBody = rawLateral * MAX_TARGET_SPEED_MM_S * dt;

                double cosH = Math.cos(currentHeading);
                double sinH = Math.sin(currentHeading);
                targetX += (cosH * dXBody - sinH * dYBody);
                targetY += (sinH * dXBody + cosH * dYBody);
            }

            if (Math.abs(rawYaw) > 0.01) {
                targetHeading += rawYaw * 2.0 * dt;
                targetHeading = normalizeAngle(targetHeading);
            }

            double errX = targetX - currentX;
            double errY = targetY - currentY;
            double distanceError = Math.hypot(errX, errY);

            double axialPower = 0.0;
            double lateralPower = 0.0;

            if (distanceError > 3.0) {
                double speedMag = Math.min(MAX_CHASSIS_POWER, distanceError * POSITION_P_GAIN);
                double vxField = (errX / distanceError) * speedMag;
                double vyField = (errY / distanceError) * speedMag;

                double cosH = Math.cos(currentHeading);
                double sinH = Math.sin(currentHeading);
                axialPower = cosH * vxField + sinH * vyField;
                lateralPower = sinH * vxField - cosH * vyField;
            }

            double headingError = normalizeAngle(targetHeading - currentHeading);
            double yawPower = 0.0;
            if (Math.abs(headingError) > Math.toRadians(1.0)) {
                yawPower = Math.max(-0.4, Math.min(0.4, headingError * HEADING_KP));
            }

            driveMecanum(axialPower, lateralPower, yawPower);

            telemetry.addData("【位置追踪】", "当前: X=%.0f mm, Y=%.0f mm", currentX, currentY);
            telemetry.addData("【目标位置】", "目标: X=%.0f mm, Y=%.0f mm | 距目标: %.1f mm", targetX, targetY, distanceError);
            telemetry.addData("【航向对比】", "当前角度: %.1f° | 目标角度: %.1f°", Math.toDegrees(currentHeading), Math.toDegrees(targetHeading));
            telemetry.addData("【Pinpoint状态】", "%s", pinpoint.getDeviceStatus());
            telemetry.update();
        }

        stopAll();
    }

    public double[] driveMecanum(double rawAxial, double rawLateral, double rawYaw) {
        double axial = AXIAL_SIGN * rawAxial;
        double lateral = LATERAL_SIGN * rawLateral;
        double yaw = YAW_SIGN * rawYaw;

        if (SWAP_LATERAL_YAW) {
            double temp = lateral;
            lateral = yaw;
            yaw = temp;
        }

        double lf = axial + lateral + yaw;
        double rf = axial - lateral - yaw;
        double lb = axial - lateral + yaw;
        double rb = axial + lateral - yaw;

        double max = Math.max(Math.max(Math.abs(lf), Math.abs(rf)),
                Math.max(Math.abs(lb), Math.abs(rb)));
        if (max > 1.0) {
            lf /= max;
            rf /= max;
            lb /= max;
            rb /= max;
        }

        frontLeft.setPower(lf);
        frontRight.setPower(rf);
        backLeft.setPower(lb);
        backRight.setPower(rb);

        return new double[]{lf, rf, lb, rb};
    }

    public void stopAll() {
        if (frontLeft != null) frontLeft.setPower(0);
        if (frontRight != null) frontRight.setPower(0);
        if (backLeft != null) backLeft.setPower(0);
        if (backRight != null) backRight.setPower(0);
    }

    private void configureChassisMotor(DcMotor motor, DcMotor.Direction direction) {
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(CHASSIS_ZERO_POWER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(0);
    }

    private double curve(double raw) {
        double magnitude = Math.abs(raw);
        if (magnitude <= DEADZONE) return 0.0;
        double scaled = (magnitude - DEADZONE) / (1.0 - DEADZONE);
        return Math.copySign(Math.min(scaled, 1.0), raw);
    }

    private double normalizeAngle(double radians) {
        while (radians > Math.PI) radians -= 2 * Math.PI;
        while (radians <= -Math.PI) radians += 2 * Math.PI;
        return radians;
    }

    private DcMotor findMotor(String position, String... wantedNames) {
        DcMotor motor = findMotorOrNull(wantedNames);
        if (motor != null) return motor;
        throw new IllegalStateException(
                "找不到" + position + "电机：已尝试 " + Arrays.toString(wantedNames)
                        + "，但机器人配置里只有 " + hardwareMap.getAllNames(DcMotor.class)
                        + "。请到 Driver Station → Configure Robot 核对。");
    }

    private DcMotor findMotorOrNull(String... wantedNames) {
        for (String wanted : wantedNames) {
            DcMotor motor = hardwareMap.tryGet(DcMotor.class, wanted);
            if (motor != null) return motor;
        }
        for (DcMotor motor : hardwareMap.getAll(DcMotor.class)) {
            for (String configuredName : hardwareMap.getNamesOf(motor)) {
                for (String wanted : wantedNames) {
                    if (configuredName.trim().equalsIgnoreCase(wanted)) return motor;
                }
            }
        }
        return null;
    }
}