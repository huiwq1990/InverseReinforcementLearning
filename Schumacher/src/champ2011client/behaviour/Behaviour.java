package champ2011client.behaviour;

import champ2011client.Action;
import champ2011client.SensorModel;

/**
 * Interface for a behaviour. A behaviour is anything which is able to execute
 * an action concerning the control of the car. A controller can use one or more
 * behaviours to define its actions.
 */
public interface Behaviour {    

    /**
     * Executes this behaviour.
     *
     * @param data The current sensor data.
     * @param action An action object, which gets modified by this behaviour.
     */
    public void execute(final SensorModel data, Action action);

    public void reset();

    public void shutdown();
}
