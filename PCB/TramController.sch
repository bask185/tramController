EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Connector:Screw_Terminal_01x02 J2
U 1 1 60BDA717
P 4750 7000
F 0 "J2" V 4950 6850 50  0000 R CNN
F 1 "power" H 4950 7150 50  0000 R CNN
F 2 "TerminalBlock_Phoenix:TerminalBlock_Phoenix_MKDS-1,5-2-5.08_1x02_P5.08mm_Horizontal" H 4750 7000 50  0001 C CNN
F 3 "~" H 4750 7000 50  0001 C CNN
	1    4750 7000
	-1   0    0    -1  
$EndComp
Wire Wire Line
	6900 3950 7000 3950
$Comp
L power:GND #PWR0105
U 1 1 60BDFF9A
P 7000 4300
F 0 "#PWR0105" H 7000 4050 50  0001 C CNN
F 1 "GND" H 7005 4127 50  0000 C CNN
F 2 "" H 7000 4300 50  0001 C CNN
F 3 "" H 7000 4300 50  0001 C CNN
	1    7000 4300
	1    0    0    -1  
$EndComp
Wire Wire Line
	7000 3950 7000 4300
$Comp
L Relay:SANYOU_SRD_Form_C K1
U 1 1 60BE9911
P 1600 3000
F 0 "K1" H 2150 3200 50  0000 R CNN
F 1 "SPST" H 2200 3100 50  0000 R CNN
F 2 "Relay_THT:Relay_SPDT_SANYOU_SRD_Series_Form_C" H 2050 2950 50  0001 L CNN
F 3 "http://www.sanyourelay.ca/public/products/pdf/SRD.pdf" H 1600 3000 50  0001 C CNN
	1    1600 3000
	0    -1   -1   0   
$EndComp
$Comp
L Transistor_BJT:BC547 Q1
U 1 1 60BEC0F5
P 2250 3400
F 0 "Q1" H 2000 3250 50  0000 L CNN
F 1 "BC547" H 2000 3150 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-92_Inline" H 2450 3325 50  0001 L CIN
F 3 "http://www.fairchildsemi.com/ds/BC/BC547.pdf" H 2250 3400 50  0001 L CNN
	1    2250 3400
	-1   0    0    -1  
$EndComp
$Comp
L Device:D D3
U 1 1 60BECF6E
P 1600 3600
F 0 "D3" V 1554 3680 50  0000 L CNN
F 1 "D" V 1645 3680 50  0000 L CNN
F 2 "Diode_THT:D_A-405_P7.62mm_Horizontal" H 1600 3600 50  0001 C CNN
F 3 "~" H 1600 3600 50  0001 C CNN
	1    1600 3600
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0107
U 1 1 60BF5663
P 7100 1850
F 0 "#PWR0107" H 7100 1700 50  0001 C CNN
F 1 "+5V" H 7115 2023 50  0000 C CNN
F 2 "" H 7100 1850 50  0001 C CNN
F 3 "" H 7100 1850 50  0001 C CNN
	1    7100 1850
	1    0    0    -1  
$EndComp
NoConn ~ 6400 2450
NoConn ~ 6400 2350
Text Label 7550 3650 0    50   ~ 0
speed
Wire Wire Line
	10450 5100 10450 5200
$Comp
L power:GND #PWR0108
U 1 1 60C027C9
P 10450 5200
F 0 "#PWR0108" H 10450 4950 50  0001 C CNN
F 1 "GND" H 10455 5027 50  0000 C CNN
F 2 "" H 10450 5200 50  0001 C CNN
F 3 "" H 10450 5200 50  0001 C CNN
	1    10450 5200
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0109
U 1 1 60C05BDE
P 2150 3650
F 0 "#PWR0109" H 2150 3400 50  0001 C CNN
F 1 "GND" H 2155 3477 50  0000 C CNN
F 2 "" H 2150 3650 50  0001 C CNN
F 3 "" H 2150 3650 50  0001 C CNN
	1    2150 3650
	1    0    0    -1  
$EndComp
Text Label 1000 4200 0    50   ~ 0
blue
Text Label 1000 3900 0    50   ~ 0
frontTrack
Text Label 1000 4000 0    50   ~ 0
rearTrack
$Comp
L Device:R R1
U 1 1 60C193B9
P 2700 3400
F 0 "R1" V 2493 3400 50  0000 C CNN
F 1 "1k" V 2584 3400 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P7.62mm_Horizontal" V 2630 3400 50  0001 C CNN
F 3 "~" H 2700 3400 50  0001 C CNN
	1    2700 3400
	0    1    1    0   
$EndComp
Wire Wire Line
	6400 3650 6050 3650
Text Label 7550 3350 0    50   ~ 0
auto_manual
Text Label 9950 4900 0    50   ~ 0
sw1
Text Label 9950 5000 0    50   ~ 0
sw2
NoConn ~ 7400 2350
NoConn ~ 7400 2450
NoConn ~ 7400 2750
Text Label 10000 4100 2    50   ~ 0
status1
Text Label 6200 2950 2    50   ~ 0
status1
Wire Wire Line
	6400 2850 6200 2850
$Comp
L Device:R R6
U 1 1 60C73131
P 10150 4300
F 0 "R6" H 10300 4350 50  0000 R CNN
F 1 "330R" H 10400 4250 50  0000 R CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P7.62mm_Horizontal" V 10080 4300 50  0001 C CNN
F 3 "~" H 10150 4300 50  0001 C CNN
	1    10150 4300
	1    0    0    -1  
$EndComp
Text Label 10150 4100 0    50   ~ 0
status2
Text Label 6200 2850 2    50   ~ 0
status2
Wire Wire Line
	6400 2950 6200 2950
Text Label 6200 2750 2    50   ~ 0
IN1
Text Label 6200 2650 2    50   ~ 0
IN2
Text Label 6200 2550 2    50   ~ 0
enable
$Comp
L Device:D_Bridge_+A-A D1
U 1 1 60C7B031
P 1950 1450
F 0 "D1" H 2200 1750 50  0000 R CNN
F 1 "d" H 2150 1650 50  0000 R CNN
F 2 "Diode_THT:Diode_Bridge_Round_D9.0mm" H 1950 1450 50  0001 C CNN
F 3 "~" H 1950 1450 50  0001 C CNN
	1    1950 1450
	-1   0    0    1   
$EndComp
$Comp
L Device:D_Bridge_+A-A D2
U 1 1 60C7B9B4
P 1600 6700
F 0 "D2" V 1500 7200 50  0000 R CNN
F 1 "D_Bridge_+A-A" V 1400 7400 50  0000 R CNN
F 2 "Diode_THT:Diode_Bridge_Round_D9.0mm" H 1600 6700 50  0001 C CNN
F 3 "~" H 1600 6700 50  0001 C CNN
	1    1600 6700
	0    -1   -1   0   
$EndComp
Wire Wire Line
	950  4100 1000 4100
Wire Wire Line
	2250 1450 2250 1500
Wire Wire Line
	2250 1500 1650 1500
Wire Wire Line
	1650 1500 1650 1450
$Comp
L Isolator:EL814 U1
U 1 1 60C94ADD
P 1250 1500
F 0 "U1" H 1250 1825 50  0000 C CNN
F 1 "EL814" H 1250 1734 50  0000 C CNN
F 2 "Package_DIP:DIP-4_W7.62mm" H 1050 1300 50  0001 L CIN
F 3 "http://www.everlight.com/file/ProductFile/EL814.pdf" H 1275 1500 50  0001 L CNN
	1    1250 1500
	-1   0    0    -1  
$EndComp
Wire Wire Line
	1950 1750 1550 1750
Wire Wire Line
	1550 1750 1550 1600
Wire Wire Line
	1950 1100 1950 1150
$Comp
L power:GND #PWR0116
U 1 1 60CBD3D5
P 950 1700
F 0 "#PWR0116" H 950 1450 50  0001 C CNN
F 1 "GND" H 955 1527 50  0000 C CNN
F 2 "" H 950 1700 50  0001 C CNN
F 3 "" H 950 1700 50  0001 C CNN
	1    950  1700
	1    0    0    -1  
$EndComp
Wire Wire Line
	950  1700 950  1600
Wire Wire Line
	6400 3050 6200 3050
Text Label 6200 3050 2    50   ~ 0
detector
Text Label 600  1400 0    50   ~ 0
detector
Wire Wire Line
	1600 6400 1650 6400
Wire Wire Line
	2100 6700 1900 6700
$Comp
L Device:R R7
U 1 1 60D016DF
P 1750 1100
F 0 "R7" V 1543 1100 50  0000 C CNN
F 1 "33R" V 1634 1100 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P7.62mm_Horizontal" V 1680 1100 50  0001 C CNN
F 3 "~" H 1750 1100 50  0001 C CNN
	1    1750 1100
	0    1    1    0   
$EndComp
Wire Wire Line
	1550 1400 1550 1100
Wire Wire Line
	1550 1100 1600 1100
Wire Wire Line
	1900 1100 1950 1100
Connection ~ 1950 1100
$Comp
L w_connectors:HEADER_3 J4
U 1 1 6129973E
P 1000 4750
F 0 "J4" H 1128 4803 60  0000 L CNN
F 1 "servo" H 1128 4697 60  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 1000 4750 60  0001 C CNN
F 3 "" H 1000 4750 60  0000 C CNN
	1    1000 4750
	-1   0    0    1   
$EndComp
$Comp
L w_connectors:HEADER_3 J5
U 1 1 61299EA4
P 1000 5300
F 0 "J5" H 1128 5353 60  0000 L CNN
F 1 "servo" H 1128 5247 60  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 1000 5300 60  0001 C CNN
F 3 "" H 1000 5300 60  0000 C CNN
	1    1000 5300
	-1   0    0    1   
$EndComp
Wire Wire Line
	1100 5200 1300 5200
Wire Wire Line
	1100 4650 1300 4650
$Comp
L power:GND #PWR0104
U 1 1 612AAB7A
P 1550 5400
F 0 "#PWR0104" H 1550 5150 50  0001 C CNN
F 1 "GND" H 1555 5227 50  0000 C CNN
F 2 "" H 1550 5400 50  0001 C CNN
F 3 "" H 1550 5400 50  0001 C CNN
	1    1550 5400
	-1   0    0    -1  
$EndComp
$Comp
L power:GND #PWR0117
U 1 1 612AB2D0
P 1550 4850
F 0 "#PWR0117" H 1550 4600 50  0001 C CNN
F 1 "GND" H 1555 4677 50  0000 C CNN
F 2 "" H 1550 4850 50  0001 C CNN
F 3 "" H 1550 4850 50  0001 C CNN
	1    1550 4850
	-1   0    0    -1  
$EndComp
Wire Wire Line
	1550 4850 1100 4850
Wire Wire Line
	1550 4750 1100 4750
Wire Wire Line
	1550 5300 1100 5300
Wire Wire Line
	1550 5400 1100 5400
Text Label 1300 5200 2    50   ~ 0
servo1
Text Label 1300 4650 2    50   ~ 0
servo2
Text Label 6200 3150 2    50   ~ 0
servo2
Text Label 6200 3250 2    50   ~ 0
servo1
Wire Wire Line
	6200 3150 6400 3150
Wire Wire Line
	6400 3250 6200 3250
$Comp
L Device:R R8
U 1 1 612D264F
P 1600 6250
F 0 "R8" V 1393 6250 50  0000 C CNN
F 1 "4k7" V 1484 6250 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P7.62mm_Horizontal" V 1530 6250 50  0001 C CNN
F 3 "~" H 1600 6250 50  0001 C CNN
	1    1600 6250
	0    1    1    0   
$EndComp
Wire Wire Line
	1300 6700 1300 6250
Wire Wire Line
	1300 6250 1450 6250
Wire Wire Line
	1750 6250 1900 6250
Wire Wire Line
	1900 6250 1900 6700
Connection ~ 1900 6700
$Comp
L Device:R_POT RV2
U 1 1 612DCA82
P 10800 2200
F 0 "RV2" H 10730 2154 50  0000 R CNN
F 1 "R_POT" H 10730 2245 50  0000 R CNN
F 2 "Potentiometer_THT:Potentiometer_Runtron_RM-065_Vertical" H 10800 2200 50  0001 C CNN
F 3 "~" H 10800 2200 50  0001 C CNN
	1    10800 2200
	-1   0    0    1   
$EndComp
$Comp
L power:+5V #PWR0119
U 1 1 612DCA88
P 10800 2000
F 0 "#PWR0119" H 10800 1850 50  0001 C CNN
F 1 "+5V" H 10815 2173 50  0000 C CNN
F 2 "" H 10800 2000 50  0001 C CNN
F 3 "" H 10800 2000 50  0001 C CNN
	1    10800 2000
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0120
U 1 1 612DCA8E
P 10800 2550
F 0 "#PWR0120" H 10800 2300 50  0001 C CNN
F 1 "GND" H 10805 2377 50  0000 C CNN
F 2 "" H 10800 2550 50  0001 C CNN
F 3 "" H 10800 2550 50  0001 C CNN
	1    10800 2550
	1    0    0    -1  
$EndComp
Wire Wire Line
	10800 2050 10800 2000
$Comp
L Device:C C4
U 1 1 612DCA95
P 10450 2350
F 0 "C4" H 10565 2396 50  0000 L CNN
F 1 "C" H 10565 2305 50  0000 L CNN
F 2 "Capacitor_THT:C_Disc_D3.0mm_W1.6mm_P2.50mm" H 10488 2200 50  0001 C CNN
F 3 "~" H 10450 2350 50  0001 C CNN
	1    10450 2350
	1    0    0    -1  
$EndComp
Wire Wire Line
	10450 2500 10800 2500
Wire Wire Line
	10800 2350 10800 2500
Wire Wire Line
	10800 2550 10800 2500
Connection ~ 10800 2500
Wire Wire Line
	10450 2200 10650 2200
$Comp
L Device:R_POT RV3
U 1 1 612E2123
P 10750 3250
F 0 "RV3" H 10680 3204 50  0000 R CNN
F 1 "R_POT" H 10680 3295 50  0000 R CNN
F 2 "Potentiometer_THT:Potentiometer_Runtron_RM-065_Vertical" H 10750 3250 50  0001 C CNN
F 3 "~" H 10750 3250 50  0001 C CNN
	1    10750 3250
	-1   0    0    1   
$EndComp
$Comp
L power:+5V #PWR0121
U 1 1 612E2129
P 10750 3050
F 0 "#PWR0121" H 10750 2900 50  0001 C CNN
F 1 "+5V" H 10765 3223 50  0000 C CNN
F 2 "" H 10750 3050 50  0001 C CNN
F 3 "" H 10750 3050 50  0001 C CNN
	1    10750 3050
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0122
U 1 1 612E212F
P 10750 3600
F 0 "#PWR0122" H 10750 3350 50  0001 C CNN
F 1 "GND" H 10755 3427 50  0000 C CNN
F 2 "" H 10750 3600 50  0001 C CNN
F 3 "" H 10750 3600 50  0001 C CNN
	1    10750 3600
	1    0    0    -1  
$EndComp
Wire Wire Line
	10750 3100 10750 3050
$Comp
L Device:C C5
U 1 1 612E2136
P 10400 3400
F 0 "C5" H 10515 3446 50  0000 L CNN
F 1 "C" H 10515 3355 50  0000 L CNN
F 2 "Capacitor_THT:C_Disc_D3.0mm_W1.6mm_P2.50mm" H 10438 3250 50  0001 C CNN
F 3 "~" H 10400 3400 50  0001 C CNN
	1    10400 3400
	1    0    0    -1  
$EndComp
Wire Wire Line
	10400 3550 10750 3550
Wire Wire Line
	10750 3400 10750 3550
Wire Wire Line
	10750 3600 10750 3550
Connection ~ 10750 3550
$Comp
L Device:R_POT RV1
U 1 1 612E8261
P 10800 1100
F 0 "RV1" H 10730 1054 50  0000 R CNN
F 1 "R_POT" H 10730 1145 50  0000 R CNN
F 2 "Potentiometer_THT:Potentiometer_Runtron_RM-065_Vertical" H 10800 1100 50  0001 C CNN
F 3 "~" H 10800 1100 50  0001 C CNN
	1    10800 1100
	-1   0    0    1   
$EndComp
$Comp
L power:+5V #PWR0123
U 1 1 612E8267
P 10800 900
F 0 "#PWR0123" H 10800 750 50  0001 C CNN
F 1 "+5V" H 10815 1073 50  0000 C CNN
F 2 "" H 10800 900 50  0001 C CNN
F 3 "" H 10800 900 50  0001 C CNN
	1    10800 900 
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0124
U 1 1 612E826D
P 10800 1450
F 0 "#PWR0124" H 10800 1200 50  0001 C CNN
F 1 "GND" H 10805 1277 50  0000 C CNN
F 2 "" H 10800 1450 50  0001 C CNN
F 3 "" H 10800 1450 50  0001 C CNN
	1    10800 1450
	1    0    0    -1  
$EndComp
Wire Wire Line
	10800 950  10800 900 
$Comp
L Device:C C3
U 1 1 612E8274
P 10450 1250
F 0 "C3" H 10565 1296 50  0000 L CNN
F 1 "C" H 10565 1205 50  0000 L CNN
F 2 "Capacitor_THT:C_Disc_D3.0mm_W1.6mm_P2.50mm" H 10488 1100 50  0001 C CNN
F 3 "~" H 10450 1250 50  0001 C CNN
	1    10450 1250
	1    0    0    -1  
$EndComp
Wire Wire Line
	10450 1400 10800 1400
Wire Wire Line
	10800 1250 10800 1400
Wire Wire Line
	10800 1450 10800 1400
Connection ~ 10800 1400
Wire Wire Line
	10450 1100 10650 1100
Text Label 10150 2200 0    50   ~ 0
runout2
Text Label 10100 1100 0    50   ~ 0
runout1
Text Label 10100 3250 0    50   ~ 0
speed
Text Label 7550 3450 0    50   ~ 0
runout1
Text Label 7550 3550 0    50   ~ 0
runout2
Text Label 2100 6700 0    50   ~ 0
redIn
NoConn ~ 7400 2950
$Comp
L Device:R R4
U 1 1 612D6118
P 10000 4300
F 0 "R4" H 9850 4350 50  0000 C CNN
F 1 "330R" H 9850 4250 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P7.62mm_Horizontal" V 9930 4300 50  0001 C CNN
F 3 "~" H 10000 4300 50  0001 C CNN
	1    10000 4300
	1    0    0    -1  
$EndComp
Text Label 10200 4600 0    50   ~ 0
led1
Text Label 10200 4700 0    50   ~ 0
led2
Wire Wire Line
	1950 1100 1950 600 
Text Label 1950 600  0    50   ~ 0
redIn
Text Label 1000 4100 0    50   ~ 0
redOut
Text Label 950  6700 0    50   ~ 0
redOut
Wire Wire Line
	950  6700 1300 6700
Connection ~ 1300 6700
Wire Wire Line
	1650 7000 1600 7000
Text Notes 900  7150 0    50   ~ 0
voltage drop for undetected track
Wire Wire Line
	2450 3400 2550 3400
Wire Wire Line
	2850 3400 3150 3400
Text Label 3150 3400 2    50   ~ 0
relayPin
Text Label 6050 3650 0    50   ~ 0
relayPin
Text Label 850  2900 0    50   ~ 0
frontTrack
Text Label 850  2700 0    50   ~ 0
rearTrack
Wire Wire Line
	950  4000 1000 4000
Wire Wire Line
	950  3900 1000 3900
Connection ~ 1950 1750
Text Label 1950 2250 0    50   ~ 0
slidings
Wire Wire Line
	1300 3200 1250 3200
Wire Wire Line
	1250 3600 1450 3600
Wire Wire Line
	2000 3600 1750 3600
Wire Wire Line
	950  4200 1000 4200
Text Notes 2100 3050 0    50   ~ 0
switch between 1 of 2 tracks\n
Text Notes 1550 5700 2    50   ~ 0
servo's for points\n
Text Label 5200 6200 0    50   ~ 0
enable
Text Label 5200 6300 0    50   ~ 0
IN2
Text Label 5200 6400 0    50   ~ 0
IN1
Wire Wire Line
	6200 2650 6400 2650
Wire Wire Line
	6200 2550 6400 2550
Wire Wire Line
	6200 2750 6400 2750
Text Notes 1300 750  0    50   ~ 0
to H-bridge
Text Notes 1150 2050 0    50   ~ 0
to side tracks for\ncurrent sense
Wire Wire Line
	7550 3150 7400 3150
Wire Wire Line
	7400 3050 7550 3050
Text Label 7550 3250 0    50   ~ 0
sw1
Text Label 7550 3150 0    50   ~ 0
sw2
Wire Wire Line
	7400 3650 7550 3650
Wire Wire Line
	7400 3450 7550 3450
Wire Wire Line
	7400 3550 7550 3550
Wire Wire Line
	7550 3250 7400 3250
Wire Wire Line
	7400 3350 7550 3350
Text Label 9950 4800 0    50   ~ 0
auto_manual
Wire Wire Line
	9950 5000 10450 5000
Wire Wire Line
	9950 4900 10450 4900
Wire Wire Line
	9950 4800 10450 4800
Wire Wire Line
	10150 4600 10150 4450
Wire Wire Line
	10150 4600 10450 4600
Wire Wire Line
	10000 4700 10000 4450
$Comp
L Device:R R5
U 1 1 6169834E
P 2400 1450
F 0 "R5" H 2250 1450 50  0000 C CNN
F 1 "4k7" H 2250 1550 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P7.62mm_Horizontal" V 2330 1450 50  0001 C CNN
F 3 "~" H 2400 1450 50  0001 C CNN
	1    2400 1450
	-1   0    0    1   
$EndComp
Wire Wire Line
	2400 1600 2400 1750
Wire Wire Line
	2400 1750 1950 1750
Wire Wire Line
	1950 1100 2400 1100
Wire Wire Line
	2400 1100 2400 1300
Wire Wire Line
	1650 6400 1650 7000
Text Label 7550 3050 0    50   ~ 0
shortCircuit
Wire Wire Line
	1950 2800 1900 2800
Wire Wire Line
	1900 3200 2000 3200
Wire Wire Line
	2150 3650 2150 3600
Wire Wire Line
	1250 3600 1250 3200
Wire Wire Line
	2000 3600 2000 3200
Wire Wire Line
	10150 4150 10150 4100
Wire Wire Line
	10000 4150 10000 4100
Wire Wire Line
	10000 4700 10450 4700
Wire Wire Line
	6400 3350 6200 3350
Wire Wire Line
	6400 3450 6200 3450
Wire Wire Line
	6400 3550 6200 3550
Text Label 6200 3350 2    50   ~ 0
JP1
Text Label 6200 3450 2    50   ~ 0
JP2
Text Label 6200 3550 2    50   ~ 0
JP3
Connection ~ 7000 3950
$Comp
L MCU_Module:Arduino_Nano_v3.x A1
U 1 1 60BD2EC9
P 6900 2950
F 0 "A1" H 6900 1861 50  0000 C CNN
F 1 "Arduino_Nano_v3.x" H 6900 1770 50  0000 C CNN
F 2 "Module:Arduino_Nano" H 6900 2950 50  0001 C CIN
F 3 "http://www.mouser.com/pdfdocs/Gravitech_Arduino_Nano3_0.pdf" H 6900 2950 50  0001 C CNN
	1    6900 2950
	1    0    0    -1  
$EndComp
Wire Wire Line
	2150 3200 2000 3200
Connection ~ 2000 3200
Wire Wire Line
	850  2700 1300 2700
Wire Wire Line
	850  2900 1300 2900
Wire Wire Line
	1950 1750 1950 2800
Wire Wire Line
	600  1400 950  1400
$Comp
L power:GND #PWR0128
U 1 1 618A93AD
P 2050 4850
F 0 "#PWR0128" H 2050 4600 50  0001 C CNN
F 1 "GND" H 2055 4677 50  0000 C CNN
F 2 "" H 2050 4850 50  0001 C CNN
F 3 "" H 2050 4850 50  0001 C CNN
	1    2050 4850
	1    0    0    -1  
$EndComp
Wire Wire Line
	2050 4850 2050 4800
Wire Wire Line
	2050 4500 2050 4450
$Comp
L power:GND #PWR0130
U 1 1 618B0656
P 2050 5750
F 0 "#PWR0130" H 2050 5500 50  0001 C CNN
F 1 "GND" H 2055 5577 50  0000 C CNN
F 2 "" H 2050 5750 50  0001 C CNN
F 3 "" H 2050 5750 50  0001 C CNN
	1    2050 5750
	1    0    0    -1  
$EndComp
Wire Wire Line
	2050 5750 2050 5700
Wire Wire Line
	2050 5400 2050 5350
Wire Wire Line
	10100 3250 10400 3250
Connection ~ 10400 3250
Wire Wire Line
	10400 3250 10600 3250
Wire Wire Line
	10150 2200 10450 2200
Connection ~ 10450 2200
Wire Wire Line
	10450 1100 10100 1100
Connection ~ 10450 1100
$Comp
L Connector:Screw_Terminal_01x02 J7
U 1 1 614E47C7
P 10650 4800
F 0 "J7" V 10614 4612 50  0000 R CNN
F 1 "power" V 10523 4612 50  0000 R CNN
F 2 "TerminalBlock_Phoenix:TerminalBlock_Phoenix_MKDS-1,5-2-5.08_1x02_P5.08mm_Horizontal" H 10650 4800 50  0001 C CNN
F 3 "~" H 10650 4800 50  0001 C CNN
	1    10650 4800
	1    0    0    -1  
$EndComp
$Comp
L Connector:Screw_Terminal_01x02 J8
U 1 1 614E4D6B
P 10650 5000
F 0 "J8" V 10614 4812 50  0000 R CNN
F 1 "power" V 10523 4812 50  0000 R CNN
F 2 "TerminalBlock_Phoenix:TerminalBlock_Phoenix_MKDS-1,5-2-5.08_1x02_P5.08mm_Horizontal" H 10650 5000 50  0001 C CNN
F 3 "~" H 10650 5000 50  0001 C CNN
	1    10650 5000
	1    0    0    -1  
$EndComp
$Comp
L Connector:Screw_Terminal_01x02 J1
U 1 1 614E5055
P 750 4100
F 0 "J1" V 714 3912 50  0000 R CNN
F 1 "power" V 623 3912 50  0000 R CNN
F 2 "TerminalBlock_Phoenix:TerminalBlock_Phoenix_MKDS-1,5-2-5.08_1x02_P5.08mm_Horizontal" H 750 4100 50  0001 C CNN
F 3 "~" H 750 4100 50  0001 C CNN
	1    750  4100
	-1   0    0    -1  
$EndComp
$Comp
L Connector:Screw_Terminal_01x02 J3
U 1 1 614E70B5
P 750 3900
F 0 "J3" V 714 3712 50  0000 R CNN
F 1 "power" V 623 3712 50  0000 R CNN
F 2 "TerminalBlock_Phoenix:TerminalBlock_Phoenix_MKDS-1,5-2-5.08_1x02_P5.08mm_Horizontal" H 750 3900 50  0001 C CNN
F 3 "~" H 750 3900 50  0001 C CNN
	1    750  3900
	-1   0    0    -1  
$EndComp
$Comp
L Connector:Screw_Terminal_01x02 J6
U 1 1 612CFE00
P 10650 4700
F 0 "J6" V 10614 4512 50  0000 R CNN
F 1 "power" V 10523 4512 50  0000 R CNN
F 2 "TerminalBlock_Phoenix:TerminalBlock_Phoenix_MKDS-1,5-2-5.08_1x02_P5.08mm_Horizontal" H 10650 4700 50  0001 C CNN
F 3 "~" H 10650 4700 50  0001 C CNN
	1    10650 4700
	1    0    0    1   
$EndComp
$Comp
L Connector:Screw_Terminal_01x07 J10
U 1 1 61DD18F9
P 4750 6400
F 0 "J10" H 4668 6917 50  0000 C CNN
F 1 "H-bridge" H 4668 6826 50  0000 C CNN
F 2 "TerminalBlock_Phoenix:TerminalBlock_Phoenix_MKDS-1,5-7-5.08_1x07_P5.08mm_Horizontal" H 4750 6400 50  0001 C CNN
F 3 "~" H 4750 6400 50  0001 C CNN
	1    4750 6400
	-1   0    0    -1  
$EndComp
$Comp
L power:+12V #PWR0101
U 1 1 61DE8CC2
P 5350 7000
F 0 "#PWR0101" H 5350 6850 50  0001 C CNN
F 1 "+12V" H 5365 7173 50  0000 C CNN
F 2 "" H 5350 7000 50  0001 C CNN
F 3 "" H 5350 7000 50  0001 C CNN
	1    5350 7000
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0102
U 1 1 61DE9495
P 5350 7100
F 0 "#PWR0102" H 5350 6850 50  0001 C CNN
F 1 "GND" H 5355 6927 50  0000 C CNN
F 2 "" H 5350 7100 50  0001 C CNN
F 3 "" H 5350 7100 50  0001 C CNN
	1    5350 7100
	1    0    0    -1  
$EndComp
Wire Wire Line
	4950 7000 5350 7000
Wire Wire Line
	4950 7100 5000 7100
Wire Wire Line
	4950 6700 4950 7000
Wire Wire Line
	4950 6600 5000 6600
Wire Wire Line
	5000 6600 5000 7100
Connection ~ 5000 7100
Wire Wire Line
	5000 7100 5350 7100
$Comp
L power:+5V #PWR0103
U 1 1 61E0ECB1
P 5550 6500
F 0 "#PWR0103" H 5550 6350 50  0001 C CNN
F 1 "+5V" H 5565 6673 50  0000 C CNN
F 2 "" H 5550 6500 50  0001 C CNN
F 3 "" H 5550 6500 50  0001 C CNN
	1    5550 6500
	1    0    0    -1  
$EndComp
Wire Wire Line
	4950 6500 5550 6500
Wire Wire Line
	4950 6400 5200 6400
Wire Wire Line
	4950 6300 5200 6300
Wire Wire Line
	4950 6200 5200 6200
Text Label 5550 6100 2    50   ~ 0
shortCircuit
Wire Wire Line
	4950 6100 5550 6100
Text Label 4050 5350 1    50   ~ 0
blue
Wire Wire Line
	4050 5350 4050 5550
Text Label 4150 5350 1    50   ~ 0
redIn
Wire Wire Line
	4150 5350 4150 5550
$Comp
L Connector:Screw_Terminal_01x02 J9
U 1 1 61E6BB82
P 4050 5750
F 0 "J9" V 4250 5600 50  0000 R CNN
F 1 "power" H 4250 5900 50  0000 R CNN
F 2 "TerminalBlock_Phoenix:TerminalBlock_Phoenix_MKDS-1,5-2-5.08_1x02_P5.08mm_Horizontal" H 4050 5750 50  0001 C CNN
F 3 "~" H 4050 5750 50  0001 C CNN
	1    4050 5750
	0    -1   1    0   
$EndComp
Wire Notes Line
	4450 6850 3800 6850
Wire Notes Line
	3800 6850 3800 6000
Wire Notes Line
	3800 6000 4450 6000
Wire Notes Line
	4450 6000 4450 6850
Text Notes 4000 6350 0    50   ~ 0
L298N
NoConn ~ 7000 1950
NoConn ~ 6800 1950
Wire Wire Line
	7100 1950 7100 1850
Connection ~ 4950 7000
$Comp
L Device:CP C1
U 1 1 61EE7C3B
P 2050 4650
F 0 "C1" H 2168 4696 50  0000 L CNN
F 1 "CP" H 2168 4605 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D4.0mm_P1.50mm" H 2088 4500 50  0001 C CNN
F 3 "~" H 2050 4650 50  0001 C CNN
	1    2050 4650
	1    0    0    -1  
$EndComp
$Comp
L Device:CP C2
U 1 1 61EE88AE
P 2050 5550
F 0 "C2" H 2168 5596 50  0000 L CNN
F 1 "CP" H 2168 5505 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D4.0mm_P1.50mm" H 2088 5400 50  0001 C CNN
F 3 "~" H 2050 5550 50  0001 C CNN
	1    2050 5550
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0106
U 1 1 61EE8D86
P 1550 4750
F 0 "#PWR0106" H 1550 4600 50  0001 C CNN
F 1 "+5V" H 1565 4923 50  0000 C CNN
F 2 "" H 1550 4750 50  0001 C CNN
F 3 "" H 1550 4750 50  0001 C CNN
	1    1550 4750
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0110
U 1 1 61EE9385
P 1550 5300
F 0 "#PWR0110" H 1550 5150 50  0001 C CNN
F 1 "+5V" H 1565 5473 50  0000 C CNN
F 2 "" H 1550 5300 50  0001 C CNN
F 3 "" H 1550 5300 50  0001 C CNN
	1    1550 5300
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0111
U 1 1 61EE96BD
P 2050 5350
F 0 "#PWR0111" H 2050 5200 50  0001 C CNN
F 1 "+5V" H 2065 5523 50  0000 C CNN
F 2 "" H 2050 5350 50  0001 C CNN
F 3 "" H 2050 5350 50  0001 C CNN
	1    2050 5350
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0113
U 1 1 61EE99F6
P 2050 4450
F 0 "#PWR0113" H 2050 4300 50  0001 C CNN
F 1 "+5V" H 2065 4623 50  0000 C CNN
F 2 "" H 2050 4450 50  0001 C CNN
F 3 "" H 2050 4450 50  0001 C CNN
	1    2050 4450
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0112
U 1 1 61F1560A
P 1250 3200
F 0 "#PWR0112" H 1250 3050 50  0001 C CNN
F 1 "+5V" H 1265 3373 50  0000 C CNN
F 2 "" H 1250 3200 50  0001 C CNN
F 3 "" H 1250 3200 50  0001 C CNN
	1    1250 3200
	1    0    0    -1  
$EndComp
Connection ~ 1250 3200
$EndSCHEMATC
