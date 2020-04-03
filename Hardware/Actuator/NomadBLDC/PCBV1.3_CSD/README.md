# File structure
* Directory - KICAD
** Contains all files for the kicad project
* Directory - KICAD_READERS
** python macros that generated BOM lists for PCBA
* Directory - GERBERS
** plots for PCBA

# Updates from original BK board
* converted from eagle to kicad
* re-routed nearly all traces
* added [CSD88599Q5DC](http://www.ti.com/lit/ds/symlink/csd88599q5dc.pdf?HQS=TI-null-null-mousermode-df-pf-null-wwe&DCM=yes&ref_url=https%3A%2F%2Fwww.mouser.com%2F&distId=26) half-bridge FET
* added more caps
* added power terminal tabs for easier soldering
* added jumper to enable/disable 120 ohm CAN terminating resistor
* added USB connector
* added I/O pin to programming connector
* simplified mask to read all components
