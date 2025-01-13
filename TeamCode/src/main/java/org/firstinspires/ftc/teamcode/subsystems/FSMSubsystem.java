package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import java.util.HashMap;
import java.util.Map;

public class FSMSubsystem extends SubsystemBase {





    ////////////////////////
    // Extendo System FSM //
    ////////////////////////

    Map<ExtendoCommand, ExtendoState[]> transition = new HashMap<ExtendoCommand, ExtendoState[]>()
    {
        {
            put(ExtendoCommand.RETRACT_HORIZONTAL,
                    new ExtendoState[]{
                            ExtendoState.HORIZONTALLY_EXTENDED,
                            ExtendoState.HORIZONTALLY_RETRACTING
                    }
            );
            put(ExtendoCommand.EXTEND_HORIZONTAL,
                    new ExtendoState[]{
                            ExtendoState.RETRACTED,
                            ExtendoState.HORIZONTALLY_EXTENDING
                    }
            );
            put(ExtendoCommand.RETRACT_VERTICAL,
                    new ExtendoState[]{
                            ExtendoState.VERTICALLY_EXTENDED,
                            ExtendoState.VERTICALLY_RETRACTING
                    }
            );
            put(ExtendoCommand.EXTEND_VERTICAL,
                    new ExtendoState[]{
                            ExtendoState.RETRACTED,
                            ExtendoState.VERTICALLY_EXTENDING
                    }
            );
        }
    };

    enum ExtendoState {
        // Both slides retracted.
        RETRACTED(),

        // Horizontal slide is extending.
        HORIZONTALLY_EXTENDING(),

        // Horizontal slide extended, vertical slide retracted.
        HORIZONTALLY_EXTENDED(),

        // Horizontal slide is retracting.
        HORIZONTALLY_RETRACTING(),

        // Vertical slide is extending.
        VERTICALLY_EXTENDING(),

        // Vertical slide extended, horizontal slide retracted.
        VERTICALLY_EXTENDED(),

        // Vertical slide is retracting.
        VERTICALLY_RETRACTING();

        static public final Integer length = 1 + VERTICALLY_RETRACTING.ordinal();

        ExtendoState() {

        }

    }

    enum ExtendoCommand {
        // Retract horizontal slide.
        RETRACT_HORIZONTAL(),

        // Extend horizontal slide.
        EXTEND_HORIZONTAL(),

        // Retract vertical slide.
        RETRACT_VERTICAL(),

        // Extend vertical slide.
        EXTEND_VERTICAL();

        static public final Integer length = 1 + EXTEND_VERTICAL.ordinal();

        ExtendoCommand() {}

    }


}
