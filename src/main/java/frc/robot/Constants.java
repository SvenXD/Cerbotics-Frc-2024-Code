package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;

public class Constants {

    public static final boolean tuningMode = true;

    public static class Shooter{

        /* IDs */

        public static final int UPPER_SHOOTER_ID;
        public static final int LOWER_SHOOTER_ID;

        /*Physical Measurements */

        public static InvertedValue UPPER_SHOOTER_INVERSION;
        public static InvertedValue LOWER_SHOOTER_INVERSION;

        static{
            UPPER_SHOOTER_ID = 13;
            LOWER_SHOOTER_ID = 14;

            UPPER_SHOOTER_INVERSION = InvertedValue.Clockwise_Positive;
            LOWER_SHOOTER_INVERSION = InvertedValue.Clockwise_Positive;
        }
    }

}
