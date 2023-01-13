package org.firstinspires.ftc.teamcode;

import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.ThermalEquilibrium.homeostasis.*;
@Config
public class RobotConstants {
    public static int MAGIC_NUMBER = 42;
    public  static PIDFCoefficients LAUNCH_PID = new PIDFCoefficients();
}
