; Offline fixture: the second nonzero move uses a lower acceleration.
G90
G1 X4.125 Y5.25 Z0.5 F1800
M204 P850 T420
G1 X5.75 Y5.25 F4800
G91
G1 X-1 Y1 Z-.02
G4 P117
M400
