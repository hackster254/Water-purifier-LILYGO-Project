EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "WATER PURIFIER PROJECT"
Date "2021-09-12"
Rev "V1"
Comp "JKUAT"
Comment1 "LILYGO module "
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L 12v_5v-rescue:TTGO-TCALL-SIM800-ttgo-tcall-sim800.lib U1
U 1 1 6139EB75
P 5500 3400
F 0 "U1" H 5725 5005 50  0000 C CNN
F 1 "TTGO-TCALL-SIM800" H 5725 4914 50  0000 C CNN
F 2 "TTGO-TCALL-SIM800:TTGO-TCALL-SIM800" H 5725 4823 50  0000 C CNN
F 3 "" H 5850 3200 50  0001 C CNN
	1    5500 3400
	1    0    0    -1  
$EndComp
$Comp
L Connector:Screw_Terminal_01x02 J2
U 1 1 613A0F56
P 9650 1600
F 0 "J2" H 9730 1592 50  0000 L CNN
F 1 "Screw_Terminal_01x02 Bulbs" H 9730 1501 50  0000 L CNN
F 2 "TerminalBlock:TerminalBlock_Altech_AK300-2_P5.00mm" H 9650 1600 50  0001 C CNN
F 3 "~" H 9650 1600 50  0001 C CNN
	1    9650 1600
	1    0    0    -1  
$EndComp
Wire Wire Line
	6500 3600 8150 3600
Wire Wire Line
	6500 3700 8550 3700
Wire Wire Line
	8550 3700 8550 3350
Wire Wire Line
	7050 1800 4600 1800
Wire Wire Line
	4600 1800 4600 4200
Wire Wire Line
	4600 4200 4950 4200
Wire Wire Line
	6950 2750 6950 2400
Wire Wire Line
	6950 2400 6500 2400
$Comp
L Device:R_Small_US R1
U 1 1 613AA358
P 9000 4250
F 0 "R1" H 9068 4296 50  0000 L CNN
F 1 "R_Small_US" H 9068 4205 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P5.08mm_Horizontal" H 9000 4250 50  0001 C CNN
F 3 "~" H 9000 4250 50  0001 C CNN
	1    9000 4250
	1    0    0    -1  
$EndComp
Wire Wire Line
	9000 4150 8750 4150
Wire Wire Line
	9000 4350 9450 4350
Wire Wire Line
	9450 4350 9450 3450
Wire Wire Line
	9450 3450 7250 3450
Wire Wire Line
	7250 3450 7250 3100
Wire Wire Line
	7250 2750 6950 2750
Wire Wire Line
	8750 4700 6650 4700
Wire Wire Line
	6650 4700 6650 3800
Wire Wire Line
	6650 3800 6500 3800
$Comp
L Switch:SW_Push_Dual_x2 SW1
U 1 1 613B0FC2
P 8200 4050
F 0 "SW1" H 8200 4417 50  0000 C CNN
F 1 "SW_DIP_x02" H 8200 4326 50  0000 C CNN
F 2 "Button_Switch_THT:SW_PUSH_6mm_H5mm" H 8200 4050 50  0001 C CNN
F 3 "~" H 8200 4050 50  0001 C CNN
	1    8200 4050
	1    0    0    -1  
$EndComp
Wire Wire Line
	7450 4400 7050 4400
Wire Wire Line
	8750 4050 8750 4150
Connection ~ 8750 4150
Wire Wire Line
	8750 4150 8750 4700
Wire Wire Line
	7450 4050 7450 4400
Wire Wire Line
	7450 4050 8000 4050
Wire Wire Line
	8400 4050 8750 4050
$Comp
L Connector:Screw_Terminal_01x02 J1
U 1 1 613A07F6
P 8100 1600
F 0 "J1" H 8180 1592 50  0000 L CNN
F 1 "Screw_Terminal_01x02 Pump" H 8180 1501 50  0000 L CNN
F 2 "TerminalBlock:TerminalBlock_Altech_AK300-2_P5.00mm" H 8100 1600 50  0001 C CNN
F 3 "~" H 8100 1600 50  0001 C CNN
	1    8100 1600
	1    0    0    -1  
$EndComp
$Comp
L Connector:Screw_Terminal_01x02 J5
U 1 1 613B7788
P 1000 1550
F 0 "J5" H 1080 1542 50  0000 L CNN
F 1 "INPUT VOLTAGE 12V PANEL" H 1080 1451 50  0000 L CNN
F 2 "TerminalBlock:TerminalBlock_Altech_AK300-2_P5.00mm" H 1000 1550 50  0001 C CNN
F 3 "~" H 1000 1550 50  0001 C CNN
	1    1000 1550
	1    0    0    -1  
$EndComp
$Comp
L Device:R_US R2
U 1 1 613BE303
P 3500 2400
F 0 "R2" H 3568 2446 50  0000 L CNN
F 1 "R_US" H 3568 2355 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P5.08mm_Horizontal" V 3540 2390 50  0001 C CNN
F 3 "~" H 3500 2400 50  0001 C CNN
F 4 "R" H 3500 2400 50  0001 C CNN "Spice_Primitive"
F 5 "220" H 3500 2400 50  0001 C CNN "Spice_Model"
F 6 "Y" H 3500 2400 50  0001 C CNN "Spice_Netlist_Enabled"
	1    3500 2400
	1    0    0    -1  
$EndComp
$Comp
L Connector:USB_A J4
U 1 1 613C0907
P 4100 2400
F 0 "J4" H 4157 2867 50  0000 C CNN
F 1 "USB_A" H 4157 2776 50  0000 C CNN
F 2 "Connector_USB:USB_A_Molex_67643_Horizontal" H 4250 2350 50  0001 C CNN
F 3 " ~" H 4250 2350 50  0001 C CNN
	1    4100 2400
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D2
U 1 1 613C23DD
P 3500 2800
F 0 "D2" V 3539 2682 50  0000 R CNN
F 1 "LED" V 3448 2682 50  0000 R CNN
F 2 "LED_THT:LED_D5.0mm" H 3500 2800 50  0001 C CNN
F 3 "~" H 3500 2800 50  0001 C CNN
	1    3500 2800
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0101
U 1 1 613C4A2D
P 800 2150
F 0 "#PWR0101" H 800 1900 50  0001 C CNN
F 1 "GND" H 805 1977 50  0000 C CNN
F 2 "" H 800 2150 50  0001 C CNN
F 3 "" H 800 2150 50  0001 C CNN
	1    800  2150
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0102
U 1 1 613C5388
P 1750 3100
F 0 "#PWR0102" H 1750 2850 50  0001 C CNN
F 1 "GND" H 1755 2927 50  0000 C CNN
F 2 "" H 1750 3100 50  0001 C CNN
F 3 "" H 1750 3100 50  0001 C CNN
	1    1750 3100
	1    0    0    -1  
$EndComp
Wire Wire Line
	800  1650 800  1850
Wire Wire Line
	700  1150 700  1550
Wire Wire Line
	700  1550 800  1550
Connection ~ 1750 3100
Wire Wire Line
	3500 2950 3500 3100
Wire Wire Line
	3500 2550 3500 2650
Wire Wire Line
	4400 2200 4400 1550
Wire Wire Line
	4400 1550 3500 1550
Wire Wire Line
	4000 2800 4000 3100
Wire Wire Line
	4000 3100 3500 3100
Connection ~ 3500 3100
Wire Wire Line
	4000 3100 4100 3100
Wire Wire Line
	4100 2800 4100 3100
Connection ~ 4000 3100
Wire Wire Line
	3500 2250 3500 1550
Wire Wire Line
	800  1850 1100 1850
Wire Wire Line
	1100 1750 1100 1850
Connection ~ 800  1850
Wire Wire Line
	800  1850 800  2150
Connection ~ 1100 1850
$Comp
L power:GND #PWR0103
U 1 1 61402AE0
P 800 3050
F 0 "#PWR0103" H 800 2800 50  0001 C CNN
F 1 "GND" H 805 2877 50  0000 C CNN
F 2 "" H 800 3050 50  0001 C CNN
F 3 "" H 800 3050 50  0001 C CNN
	1    800  3050
	1    0    0    -1  
$EndComp
Wire Wire Line
	800  3050 1100 3050
Connection ~ 1100 3050
Wire Wire Line
	1100 3050 1100 3100
Wire Wire Line
	7050 1800 7050 2400
$Comp
L Connector:Screw_Terminal_01x02 J3
U 1 1 614285ED
P 8000 2800
F 0 "J3" H 8080 2792 50  0000 L CNN
F 1 "vcc_gnd_RELAY" H 8080 2701 50  0000 L CNN
F 2 "TerminalBlock:TerminalBlock_Altech_AK300-2_P5.00mm" H 8000 2800 50  0001 C CNN
F 3 "~" H 8000 2800 50  0001 C CNN
	1    8000 2800
	1    0    0    -1  
$EndComp
$Comp
L Connector:Screw_Terminal_01x02 J6
U 1 1 61429053
P 9250 2850
F 0 "J6" H 9330 2842 50  0000 L CNN
F 1 "in1_in2_RELAY" H 9330 2751 50  0000 L CNN
F 2 "TerminalBlock:TerminalBlock_Altech_AK300-2_P5.00mm" H 9250 2850 50  0001 C CNN
F 3 "~" H 9250 2850 50  0001 C CNN
	1    9250 2850
	1    0    0    -1  
$EndComp
Wire Wire Line
	7800 2800 7300 2800
Wire Wire Line
	7300 2800 7300 2400
Wire Wire Line
	7300 2400 7050 2400
Connection ~ 7050 2400
Wire Wire Line
	7050 2400 7050 4400
Wire Wire Line
	7800 2900 7450 2900
Wire Wire Line
	7450 2900 7450 3100
Wire Wire Line
	7450 3100 7250 3100
Connection ~ 7250 3100
Wire Wire Line
	7250 3100 7250 2750
Wire Wire Line
	9050 2850 8750 2850
Wire Wire Line
	8750 2850 8750 3100
Wire Wire Line
	8750 3100 8150 3100
Wire Wire Line
	8150 3100 8150 3600
Wire Wire Line
	8550 3350 8850 3350
Wire Wire Line
	8850 3350 8850 2950
Wire Wire Line
	8850 2950 9050 2950
Wire Wire Line
	7800 1150 7800 1600
Wire Wire Line
	7800 1600 7900 1600
Wire Wire Line
	4100 3100 4550 3100
Wire Wire Line
	4550 3100 4550 1300
Wire Wire Line
	4550 1300 7700 1300
Wire Wire Line
	7700 1300 7700 1750
Wire Wire Line
	7700 1750 7900 1750
Wire Wire Line
	7900 1750 7900 1700
Connection ~ 4100 3100
Wire Wire Line
	9450 1600 8850 1600
Wire Wire Line
	8850 1600 8850 1150
Wire Wire Line
	8850 1150 7800 1150
Wire Wire Line
	9450 1700 9450 2000
Wire Wire Line
	9450 2000 7700 2000
Wire Wire Line
	7700 2000 7700 1750
Connection ~ 7700 1750
Text Notes 8350 2600 0    50   ~ 0
RELAY MODULE CONNECTIONS
Text Notes 9800 1200 0    50   ~ 0
UV BULB CONNECTORS
Text Notes 7950 1000 0    50   ~ 0
DC PUMP CONNECTOR
Text Notes 550  900  0    50   ~ 0
12V INPUT FROM BATTERY
Text Notes 7550 4250 0    50   ~ 0
BUTTON TO CONTROL ACTION
Wire Wire Line
	700  1150 1400 1150
Wire Wire Line
	1100 1850 1100 3050
Wire Wire Line
	1100 3100 1750 3100
Wire Wire Line
	1750 3100 2650 3100
Connection ~ 7800 1150
Connection ~ 1400 1150
Wire Wire Line
	1400 1150 7800 1150
Wire Wire Line
	1400 1150 1400 1200
Wire Wire Line
	1400 1200 1550 1200
Wire Wire Line
	3500 1300 3500 1550
Connection ~ 3500 1550
Wire Wire Line
	2650 1700 2650 3100
Connection ~ 2650 3100
Wire Wire Line
	2650 3100 3500 3100
Text Notes 2600 900  0    50   ~ 0
LM2596 REGULATOR CONNECTORS
$Comp
L Connector:Screw_Terminal_01x01 J8
U 1 1 6141974C
P 2900 1450
F 0 "J8" H 2980 1492 50  0000 L CNN
F 1 "OUT+ LM2596" H 2980 1401 50  0000 L CNN
F 2 "Connector_PinHeader_1.00mm:PinHeader_1x01_P1.00mm_Vertical" H 2900 1450 50  0001 C CNN
F 3 "~" H 2900 1450 50  0001 C CNN
	1    2900 1450
	1    0    0    -1  
$EndComp
$Comp
L Connector:Screw_Terminal_01x01 J7
U 1 1 6141BEAC
P 2050 1800
F 0 "J7" H 2130 1842 50  0000 L CNN
F 1 "IN- LM2596" H 2130 1751 50  0000 L CNN
F 2 "Connector_PinHeader_1.00mm:PinHeader_1x01_P1.00mm_Vertical" H 2050 1800 50  0001 C CNN
F 3 "~" H 2050 1800 50  0001 C CNN
	1    2050 1800
	1    0    0    -1  
$EndComp
$Comp
L Connector:Screw_Terminal_01x01 J10
U 1 1 6141E4C9
P 2900 1850
F 0 "J10" H 2980 1892 50  0000 L CNN
F 1 "OUT- LM2596" H 2980 1801 50  0000 L CNN
F 2 "Connector_PinHeader_1.00mm:PinHeader_1x01_P1.00mm_Vertical" H 2900 1850 50  0001 C CNN
F 3 "~" H 2900 1850 50  0001 C CNN
	1    2900 1850
	1    0    0    -1  
$EndComp
$Comp
L Connector:Screw_Terminal_01x01 J9
U 1 1 61416294
P 1950 1400
F 0 "J9" H 2030 1442 50  0000 L CNN
F 1 "IN+ LM2596" H 2030 1351 50  0000 L CNN
F 2 "Connector_PinHeader_1.00mm:PinHeader_1x01_P1.00mm_Vertical" H 1950 1400 50  0001 C CNN
F 3 "~" H 1950 1400 50  0001 C CNN
	1    1950 1400
	1    0    0    -1  
$EndComp
Wire Wire Line
	1750 1400 1550 1400
Wire Wire Line
	1550 1400 1550 1200
Wire Wire Line
	1850 1800 1200 1800
Wire Wire Line
	1200 1800 1200 1750
Wire Wire Line
	1200 1750 1100 1750
Wire Wire Line
	2700 1450 2700 1300
Wire Wire Line
	2700 1300 3500 1300
Wire Wire Line
	2700 1850 2700 1700
Wire Wire Line
	2700 1700 2650 1700
$EndSCHEMATC
