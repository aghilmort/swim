/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package swim.sim.uwswarm.intelli.brain.struct.artif;

/**
 *
 * @author Sherif
 */
public enum ResidualEnergyLevel {
    VERY_LOW,   // < 10%
    LOW,        // >= 10% && < 40%
    MEDIUM,     // >= 40% && < 60%
    HIGH,       // >= 60% && < 90%
    VERY_HIGH   // >= 90%
}
