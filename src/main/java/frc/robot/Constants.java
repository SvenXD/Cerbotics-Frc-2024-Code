package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;

public class Constants {

    public static final boolean tuningMode = true;

    public static class Shooter{

        /* IDs */

        public static final int UPPER_SHOOTER_ID;
        public static final int LOWER_SHOOTER_ID;

        /* SetPoints and Threesholds */

        public static final int SHOOTER_THREESHOLD;
        
        public static final int UPPER_SHOOTER_IDLE_RPM;
        public static final int LOWER_SHOOTER_IDLE_RPM;

        public static final int UPPER_SHOOTER_AMP_RPM;
        public static final int LOWER_SHOOTER_AMP_RPM;

        public static final int UPPER_SHOOTER_FEEDER_OVER_RPM;
        public static final int LOWER_SHOOTER_FEEDER_OVER_RPM;

        public static final int UPPER_SHOOTER_FEEDER_UNDER_RPM;
        public static final int LOWER_SHOOTER_FEEDER_UNDER_RPM;

        /*Physical Measurements */

        public static InvertedValue UPPER_SHOOTER_INVERSION;
        public static InvertedValue LOWER_SHOOTER_INVERSION;

        static{
            UPPER_SHOOTER_ID = 13;
            LOWER_SHOOTER_ID = 14;

            SHOOTER_THREESHOLD = 300;

            UPPER_SHOOTER_IDLE_RPM = 0;
            LOWER_SHOOTER_IDLE_RPM = 0;

            UPPER_SHOOTER_AMP_RPM = 2000;
            LOWER_SHOOTER_AMP_RPM = 500;

            UPPER_SHOOTER_FEEDER_OVER_RPM = 4500;
            LOWER_SHOOTER_FEEDER_OVER_RPM = 4500;

            UPPER_SHOOTER_FEEDER_UNDER_RPM = 6000;
            LOWER_SHOOTER_FEEDER_UNDER_RPM = 6000;

            UPPER_SHOOTER_INVERSION = InvertedValue.Clockwise_Positive;
            LOWER_SHOOTER_INVERSION = InvertedValue.Clockwise_Positive;
        }
    }

}
