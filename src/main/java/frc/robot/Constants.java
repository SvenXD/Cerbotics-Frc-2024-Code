package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class Constants {


    public static class Drive{

        public record ModuleConfig(
      int driveID,
      int turnID,
      int absoluteEncoderChannel,
      Rotation2d absoluteEncoderOffset,
      boolean turnMotorInverted) {}

      public static final String CANBU_STRING = "Swerve_Canivore";
    
      public static final int PIGEON_ID = 15;


      public static final double kTrackWidth = 0.6096;

      public static final double kWheelBase = 0.635;
  
      public static final SwerveDriveKinematics kSwerveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2 ));
    }

    public static class Shooter{
        /* IDs */

        public static final int UPPER_SHOOTER_ID = 13;
        public static final int LOWER_SHOOTER_ID = 14;

        /* SetPoints and Threesholds */

        public static final int SHOOTER_THREESHOLD = 300;
        
        public static final int UPPER_SHOOTER_IDLE_RPM = 0;
        public static final int LOWER_SHOOTER_IDLE_RPM = 0;

        public static final int UPPER_SHOOTER_AMP_RPM = 2000;
        public static final int LOWER_SHOOTER_AMP_RPM = 500;

        public static final int UPPER_SHOOTER_FEEDER_OVER_RPM = 4500;
        public static final int LOWER_SHOOTER_FEEDER_OVER_RPM = 4500;

        public static final int UPPER_SHOOTER_FEEDER_UNDER_RPM = 6000;
        public static final int LOWER_SHOOTER_FEEDER_UNDER_RPM = 6000;

        public static final int UPPER_SHOOTER_SPEAKER_RPM = 3000;
        public static final int LOWER_SHOOTER_SPEAKER_RPM = 3000;

        /*Physical Measurements */

        public static InvertedValue UPPER_SHOOTER_INVERSION = InvertedValue.Clockwise_Positive;
        public static InvertedValue LOWER_SHOOTER_INVERSION = InvertedValue.Clockwise_Positive;

        /* PID */

        public static double
        ukP = 1.4,     
        ukI = 0,        
        ukD = 0.025,    
        ukS = 0.50189,
        ukV = 0.0016926,
        ukA = 0.00059492,    
        lkP = 1.4,     
        lkI = 0,        
        lkD = 0.025,    
        lkS = 0.50189,
        lkV = 0.0016926,
        lkA = 0.00059492;

    }

      public static class Intake{
       /*ID */
        public static final int INTAKE_ID = 12;
        public static final int INTAKE_SENSOR_ID = 0;

       /*Physical Measurements */
        public static final boolean INTAKE_INVERSION = false;

    }

      public static class Arm{
        /*ID */

        public static final byte LEFT_ARM_ID = 9;
        public static final byte RIGHT_ARM_ID = 10;  
    
        public static final int ABSOLUTE_ENCODER_ID = 17;
    
        public static final double ARM_GEARBOX = 320.0 / 1.0;
    
        /* PID */
        public static double kP = 0.32,
                             kI = 0.42,
                             kD = 0.0055,
                             kFF = 0.0,
                             kMaxVelocityRadPerSecond = 1000,
                             kMaxAccelerationMetersPerSecondSquared = 1000,
                             kS = 0.013804,
                             kV = 0.00028699,
                             kA = 0.00052411,
                             kG = 0.93532,
                             kPeriod = 0.02;
                             
        /* SetPoints and Threesholds */
        public static double INTAKING_POSITION = 180.5;

        public static double SPEAKER_SCORING_POSITION = 160.0;

        public static double INIT_POSITION = 95.0;
        
        public static double IDLE_UNDER_STAGE = 170.0;
      }


      public static final class VisionConstants {

        public static final String neuralLimelight = "limelight-neural";
        public static final String tagLimelightName = "limelight-tags";
    
        public static final int main_Pipeline = 0,
                                far_Pipeline = 1,
                                close_Pipeline = 2;
      }

  /*   _________                    
    /   _____/__  __ ____   ____  
    \_____  \\  \/ // __ \ /    \ 
   /_______  /\   /\  ___/|   |  \
           \/  \_/  \___  >___|  /
                        \/     \/
           */
  }

