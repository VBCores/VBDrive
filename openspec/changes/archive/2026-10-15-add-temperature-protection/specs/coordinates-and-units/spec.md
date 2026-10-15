## MODIFIED Requirements

### Requirement: User-facing physical units

Commands, user limits, generated references and reported output-shaft state SHALL use radians, radians/second, radians/second squared and newton metres. Voltage and current SHALL use volts and amperes. `kt` SHALL be output-shaft N m/A, generator bandwidth SHALL be s^-1 without a 2π multiplier, and measured temperature registers `temp_mcu` and `temp_stator` SHALL report kelvins. Stator protection thresholds `max_temp` and `restart_temp` SHALL use kelvins. Encoder registers SHALL report raw counts. The Cyphal State timestamp SHALL be in microseconds; the current timestamp is local device time, without network clock synchronization.

#### Scenario: Temperature and state interpretation

- **WHEN** a client reads a temperature of 303.15 and velocity of 2
- **THEN** these denote 30 degrees Celsius and 2 output-shaft rad/s, independently of the transport.

#### Scenario: Kelvin threshold and measurement

- **WHEN** max_temp reads 363.15 and temp_stator reads 363.15
- **THEN** the stator is at the 363.15 K trip threshold; the measurement register has not changed units.

