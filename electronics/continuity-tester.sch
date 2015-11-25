EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:conn
LIBS:microchip_pic12mcu
LIBS:audio
LIBS:linear
LIBS:continuity-tester-cache
EELAYER 25 0
EELAYER END
$Descr USLetter 11000 8500
encoding utf-8
Sheet 1 1
Title "Continuity Tester"
Date "2015-11-17"
Rev "A"
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L PIC12(L)F1501-I/SN IC1
U 1 1 564AB02C
P 4500 3850
F 0 "IC1" H 3750 4400 50  0000 L CNN
F 1 "PIC12LF1571-I/SN" H 3750 4300 50  0000 L CNN
F 2 "Housings_SOIC:SOIC-8_3.9x4.9mm_Pitch1.27mm" H 4450 3350 60  0001 C CNN
F 3 "" H 4500 3850 60  0000 C CNN
	1    4500 3850
	1    0    0    -1  
$EndComp
$Comp
L R R5
U 1 1 564AB0FA
P 6650 3950
F 0 "R5" V 6730 3950 50  0000 C CNN
F 1 "4k7" V 6650 3950 50  0000 C CNN
F 2 "Resistors_SMD:R_0805" V 6580 3950 30  0001 C CNN
F 3 "" H 6650 3950 30  0000 C CNN
	1    6650 3950
	0    1    1    0   
$EndComp
$Comp
L R R3
U 1 1 564AB147
P 6650 3350
F 0 "R3" V 6730 3350 50  0000 C CNN
F 1 "22k" V 6650 3350 50  0000 C CNN
F 2 "Resistors_SMD:R_0805" V 6580 3350 30  0001 C CNN
F 3 "" H 6650 3350 30  0000 C CNN
	1    6650 3350
	0    1    1    0   
$EndComp
$Comp
L R R4
U 1 1 564AB186
P 6650 3650
F 0 "R4" V 6730 3650 50  0000 C CNN
F 1 "22k" V 6650 3650 50  0000 C CNN
F 2 "Resistors_SMD:R_0805" V 6580 3650 30  0001 C CNN
F 3 "" H 6650 3650 30  0000 C CNN
	1    6650 3650
	0    1    1    0   
$EndComp
$Comp
L CONN_01X05 ICSP1
U 1 1 564AB227
P 6050 2600
F 0 "ICSP1" H 6050 2900 50  0000 C CNN
F 1 "CONN_01X05" V 6150 2600 50  0001 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x05" H 6050 2600 60  0001 C CNN
F 3 "" H 6050 2600 60  0000 C CNN
	1    6050 2600
	0    -1   -1   0   
$EndComp
$Comp
L C C1
U 1 1 564AB703
P 3300 3850
F 0 "C1" H 3325 3950 50  0000 L CNN
F 1 "100n" H 3325 3750 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 3338 3700 30  0001 C CNN
F 3 "" H 3300 3850 60  0000 C CNN
	1    3300 3850
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X01 P1
U 1 1 564AB8CB
P 7650 3650
F 0 "P1" H 7650 3750 50  0000 C CNN
F 1 "CONN_01X01" V 7750 3650 50  0001 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x01" H 7650 3650 60  0001 C CNN
F 3 "" H 7650 3650 60  0000 C CNN
	1    7650 3650
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X01 P2
U 1 1 564ABA35
P 7650 5000
F 0 "P2" H 7650 5100 50  0000 C CNN
F 1 "CONN_01X01" V 7750 5000 50  0001 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x01" H 7650 5000 60  0001 C CNN
F 3 "" H 7650 5000 60  0000 C CNN
	1    7650 5000
	1    0    0    -1  
$EndComp
$Comp
L Battery BT1
U 1 1 564AC30C
P 2550 3850
F 0 "BT1" H 2650 3900 50  0000 L CNN
F 1 "CR2032" H 2650 3800 50  0000 L CNN
F 2 "Battery Holders:BAT-HLD-001-THM-OTL" V 2550 3890 60  0001 C CNN
F 3 "" V 2550 3890 60  0000 C CNN
	1    2550 3850
	1    0    0    -1  
$EndComp
$Comp
L R R1
U 1 1 564AC640
P 5500 4250
F 0 "R1" V 5580 4250 50  0000 C CNN
F 1 "4K7" V 5500 4250 50  0000 C CNN
F 2 "Resistors_SMD:R_0805" V 5430 4250 30  0001 C CNN
F 3 "" H 5500 4250 30  0000 C CNN
	1    5500 4250
	-1   0    0    1   
$EndComp
$Comp
L LED D1
U 1 1 564AC7F4
P 5500 4700
F 0 "D1" H 5500 4800 50  0000 C CNN
F 1 "LED" H 5500 4600 50  0001 C CNN
F 2 "LEDs:LED-3MM" H 5500 4700 60  0001 C CNN
F 3 "" H 5500 4700 60  0000 C CNN
	1    5500 4700
	0    -1   -1   0   
$EndComp
$Comp
L SPEAKER SP1
U 1 1 564ACB15
P 6600 4550
F 0 "SP1" H 6500 4800 50  0000 C CNN
F 1 "SPEAKER" H 6500 4300 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02" H 6600 4550 60  0001 C CNN
F 3 "" H 6600 4550 60  0000 C CNN
	1    6600 4550
	1    0    0    -1  
$EndComp
$Comp
L SW_PUSH SW1
U 1 1 564ACD7C
P 5850 4600
F 0 "SW1" H 6000 4710 50  0000 C CNN
F 1 "SW_PUSH" H 5850 4520 50  0001 C CNN
F 2 "Buttons_Switches_ThroughHole:SW_PUSH_SMALL" H 5850 4600 60  0001 C CNN
F 3 "" H 5850 4600 60  0000 C CNN
	1    5850 4600
	0    -1   -1   0   
$EndComp
$Comp
L R R2
U 1 1 564AD8EC
P 5650 3300
F 0 "R2" V 5730 3300 50  0000 C CNN
F 1 "22k" V 5650 3300 50  0000 C CNN
F 2 "Resistors_SMD:R_0805" V 5580 3300 30  0001 C CNN
F 3 "" H 5650 3300 30  0000 C CNN
	1    5650 3300
	0    1    1    0   
$EndComp
Text Notes 7500 4350 0    79   ~ 0
PROBES
Wire Wire Line
	5850 2800 5850 4300
Wire Wire Line
	5850 3850 5100 3850
Wire Wire Line
	3300 2900 3300 3700
Wire Wire Line
	3300 3550 3900 3550
Wire Wire Line
	3300 4150 3900 4150
Wire Wire Line
	6150 2800 6150 3550
Wire Wire Line
	6150 3350 6500 3350
Wire Wire Line
	6250 2800 6250 3650
Connection ~ 3300 3550
Connection ~ 3300 4150
Connection ~ 6150 3350
Connection ~ 6250 3650
Wire Wire Line
	6500 3950 5100 3950
Wire Wire Line
	6950 3950 6800 3950
Wire Wire Line
	6950 3350 6950 3950
Wire Wire Line
	6800 3350 6950 3350
Connection ~ 6950 3650
Wire Wire Line
	2550 5000 7450 5000
Connection ~ 3300 5000
Wire Wire Line
	2550 2900 5950 2900
Wire Wire Line
	2550 2900 2550 3700
Connection ~ 3300 2900
Wire Wire Line
	2550 4000 2550 5000
Wire Wire Line
	3300 4000 3300 5000
Wire Wire Line
	5100 4050 5500 4050
Wire Wire Line
	5500 4050 5500 4100
Wire Wire Line
	5500 4400 5500 4500
Wire Wire Line
	5500 4900 5500 5000
Connection ~ 5500 5000
Wire Wire Line
	6300 4650 6250 4650
Wire Wire Line
	6250 4650 6250 5000
Connection ~ 6250 5000
Wire Wire Line
	6300 4450 6250 4450
Wire Wire Line
	6250 4450 6250 3750
Wire Wire Line
	6250 3750 5100 3750
Connection ~ 5850 3850
Wire Wire Line
	5850 4900 5850 5000
Connection ~ 5850 5000
Wire Wire Line
	6150 3550 5100 3550
Wire Wire Line
	6050 2800 6050 5000
Connection ~ 6050 5000
Wire Wire Line
	5450 2900 5450 3300
Wire Wire Line
	5950 2900 5950 2800
Connection ~ 5450 2900
Wire Wire Line
	5800 3300 5850 3300
Connection ~ 5850 3300
Wire Wire Line
	5450 3300 5500 3300
Wire Wire Line
	5100 3650 6500 3650
Wire Wire Line
	6800 3650 7450 3650
$EndSCHEMATC
