# Release notes

This build is for a garden light / PIR combo.
PIR IC has a short between p3/p4 and 47kOhm between p5/p6 to reduce the activation time and the interval between activations.

Ruuvi P30 -> PIR IC p3/p4 (GRN)
Ruuvi P31 -> PIR IC p8 (WHT)


## Advertising format

* *On startup*: an empty ad with MAC, Voltage and TX power only.
* *On event*: standard packet with pressure byte indicating the number of external sensor activations (Pin 30).

## External pins

* Pin 30 - PIR controller IC to LED mosfet output, active LOW, must be 70% or less of Pin 31 voltage
* Pin 31 - Vdd of the PIR controller IC, which is on if there is no light hitting the solar panel.

## Operation

* Ruuvi is always on as long as there is enough voltage in the battery
* A 2 min timeout on powering up the PIR - no ads are sent
* No timeout between PIR activations
* Advertising for 5 sec at 200ms interval
* Ax events trigger ads regardless
