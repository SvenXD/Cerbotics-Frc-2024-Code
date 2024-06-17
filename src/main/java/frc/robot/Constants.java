package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;

public class Constants {

    public static final boolean tuningMode = true;

    public static class Shooter{

        /* IDs */

        public static final int UPPER_SHOOTER_ID =13;
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

    }

  /*   _________                    
    /   _____/__  __ ____   ____  
    \_____  \\  \/ // __ \ /    \ 
    /        \\   /\  ___/|   |  \
   /_______  / \_/  \___  >___|  /
           \/           \/     \/
          
          
           */
}
