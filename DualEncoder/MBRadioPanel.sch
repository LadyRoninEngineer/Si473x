EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr USLetter 11000 8500
encoding utf-8
Sheet 1 1
Title "Micro All Band Radio"
Date "2020-09-24"
Rev "Version 1"
Comp ""
Comment1 ""
Comment2 "Designed By: Nancy Gail Daniels"
Comment3 ""
Comment4 ""
$EndDescr
NoConn ~ 200  -1500
$Comp
L power:GND #PWR0110
U 1 1 5F52A0DE
P 6750 4950
F 0 "#PWR0110" H 6750 4700 50  0001 C CNN
F 1 "GND" H 6755 4777 50  0000 C CNN
F 2 "" H 6750 4950 50  0001 C CNN
F 3 "" H 6750 4950 50  0001 C CNN
	1    6750 4950
	-1   0    0    -1  
$EndComp
$Comp
L Device:Rotary_Encoder_Switch S3
U 1 1 5F79F1AE
P 1600 4600
F 0 "S3" V 1600 4830 50  0000 L CNN
F 1 "Rotary_Encoder_Switch" V 1645 4830 50  0001 L CNN
F 2 "Rotary_Encoder:RotaryEncoder_Alps_EC11E-Switch_Vertical_H20mm" H 1450 4760 50  0001 C CNN
F 3 "~" H 1600 4860 50  0001 C CNN
	1    1600 4600
	0    1    1    0   
$EndComp
Wire Wire Line
	1500 5100 1500 4900
Wire Wire Line
	1600 4300 1600 4200
Wire Wire Line
	1600 4200 1300 4200
Wire Wire Line
	1300 4200 1300 5100
Wire Wire Line
	1300 5100 1500 5100
Connection ~ 1500 5100
Text Notes 5890 5270 1    50   Italic 0
AGC
Text Notes 6030 5080 3    50   Italic 0
Band
Text Notes 6180 4990 3    50   Italic 0
Scan -
Text Notes 6330 4990 3    50   Italic 0
Scan +
$Comp
L Device:Rotary_Encoder_Switch S4
U 1 1 5FBC7E84
P 3600 4600
F 0 "S4" V 3600 4830 50  0000 L CNN
F 1 "Rotary_Encoder_Switch" V 3645 4830 50  0001 L CNN
F 2 "Rotary_Encoder:RotaryEncoder_Alps_EC11E-Switch_Vertical_H20mm" H 3450 4760 50  0001 C CNN
F 3 "~" H 3600 4860 50  0001 C CNN
	1    3600 4600
	0    1    1    0   
$EndComp
Wire Wire Line
	1700 4900 1700 5000
Wire Wire Line
	3500 4900 3500 5100
Wire Wire Line
	3500 5100 3300 5100
Wire Wire Line
	3600 4300 3600 4200
Wire Wire Line
	3600 4200 3300 4200
Wire Wire Line
	3300 4200 3300 5100
Connection ~ 3300 5100
Wire Wire Line
	3300 5100 2500 5100
$Comp
L New_Library:87FC3 S2
U 1 1 5F7C0D9A
P 6600 4750
F 0 "S2" H 7500 4600 50  0000 R CNN
F 1 "87FC3" H 6700 4850 39  0000 R CNN
F 2 "Grayhill_87_Series:87FC3" H 6600 4750 50  0001 C CNN
F 3 "" H 6600 4750 50  0001 C CNN
	1    6600 4750
	-1   0    0    1   
$EndComp
Wire Wire Line
	6450 4600 6450 4550
Wire Wire Line
	6450 4550 6750 4550
Wire Wire Line
	6750 4550 6750 4950
$Comp
L power:GND #PWR0112
U 1 1 5FADB80B
P 3500 5200
F 0 "#PWR0112" H 3500 4950 50  0001 C CNN
F 1 "GND" H 3505 5027 50  0000 C CNN
F 2 "" H 3500 5200 50  0001 C CNN
F 3 "" H 3500 5200 50  0001 C CNN
	1    3500 5200
	-1   0    0    -1  
$EndComp
Wire Wire Line
	3500 5100 3500 5200
$Comp
L Connector:Conn_01x12_Male J1
U 1 1 5F8E70F7
P 5550 2250
F 0 "J1" V 5650 2950 50  0000 R CNN
F 1 "Conn_01x12_Male" V 5500 1600 50  0001 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x12_P2.54mm_Vertical" H 5550 2250 50  0001 C CNN
F 3 "~" H 5550 2250 50  0001 C CNN
	1    5550 2250
	0    1    1    0   
$EndComp
Wire Wire Line
	6300 3800 6050 3800
Wire Wire Line
	6050 3800 6050 2450
Wire Wire Line
	6300 3800 6300 4600
Wire Wire Line
	6150 3900 5950 3900
Wire Wire Line
	5950 3900 5950 2450
Wire Wire Line
	6150 3900 6150 4600
Wire Wire Line
	6000 4000 5850 4000
Wire Wire Line
	5850 4000 5850 2450
Wire Wire Line
	6000 4000 6000 4600
Wire Wire Line
	5750 2450 5750 4100
Wire Wire Line
	5750 4100 5850 4100
Wire Wire Line
	5850 4100 5850 4600
Wire Wire Line
	5650 4900 5650 2450
Wire Wire Line
	3700 4900 5650 4900
Wire Wire Line
	5550 5000 5550 2450
Wire Wire Line
	1700 5000 5550 5000
Wire Wire Line
	3700 4300 3700 4200
Wire Wire Line
	3700 4200 4150 4200
Wire Wire Line
	5450 4200 5450 2450
Wire Wire Line
	5350 2450 5350 4050
Wire Wire Line
	5250 2450 5250 3900
Wire Wire Line
	1700 3900 1700 4300
Wire Wire Line
	1500 3750 1500 4300
Text Notes 5190 2250 1    50   Italic 0
ENC2B
Text Notes 5290 1990 3    50   Italic 0
ENC2A
Text Notes 5390 1980 3    50   Italic 0
ENC1B
Text Notes 5490 1990 3    50   Italic 0
ENC1A
Text Notes 5590 2240 1    50   Italic 0
ENC2S
Text Notes 5690 1990 3    50   Italic 0
ENC1S
Text Notes 5790 2080 3    50   Italic 0
PB4
Text Notes 5890 2090 3    50   Italic 0
PB3
Text Notes 5090 2080 3    50   Italic 0
GND
Text Notes 5990 2080 3    50   Italic 0
PB2
Text Notes 6090 2090 3    50   Italic 0
PB1
$Comp
L Device:R_Small R1
U 1 1 5F850DE9
P 1500 3400
F 0 "R1" H 1350 3450 50  0000 L CNN
F 1 "10kΩ" H 1250 3350 50  0000 L CNN
F 2 "Resistor_SMD:R_0402_1005Metric" H 1500 3400 50  0001 C CNN
F 3 "~" H 1500 3400 50  0001 C CNN
	1    1500 3400
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R2
U 1 1 5F853092
P 1950 3750
F 0 "R2" V 1870 3750 50  0000 C CNN
F 1 "10kΩ" V 2020 3750 50  0000 C CNN
F 2 "Resistor_SMD:R_0402_1005Metric" H 1950 3750 50  0001 C CNN
F 3 "~" H 1950 3750 50  0001 C CNN
	1    1950 3750
	0    1    1    0   
$EndComp
Wire Wire Line
	1500 3750 1850 3750
Wire Wire Line
	1500 3750 1500 3500
Connection ~ 1500 3750
Wire Wire Line
	1500 3300 1500 3200
$Comp
L Device:R_Small R3
U 1 1 5F86744A
P 1700 3400
F 0 "R3" H 1759 3446 50  0000 L CNN
F 1 "10kΩ" H 1759 3355 50  0000 L CNN
F 2 "Resistor_SMD:R_0402_1005Metric" H 1700 3400 50  0001 C CNN
F 3 "~" H 1700 3400 50  0001 C CNN
	1    1700 3400
	1    0    0    -1  
$EndComp
Wire Wire Line
	1700 3300 1700 3200
Wire Wire Line
	1700 3200 1500 3200
Connection ~ 1500 3200
Wire Wire Line
	1500 3200 1500 3100
Wire Wire Line
	1700 3500 1700 3900
Text Notes 4990 2086 3    50   Italic 0
VCC
$Comp
L power:VCC #PWR0102
U 1 1 5F87664A
P 1500 3100
F 0 "#PWR0102" H 1500 2950 50  0001 C CNN
F 1 "VCC" H 1515 3273 50  0000 C CNN
F 2 "" H 1500 3100 50  0001 C CNN
F 3 "" H 1500 3100 50  0001 C CNN
	1    1500 3100
	1    0    0    -1  
$EndComp
Wire Wire Line
	4750 2550 4750 2450
$Comp
L power:VCC #PWR0103
U 1 1 5F8809B2
P 4750 2450
F 0 "#PWR0103" H 4750 2300 50  0001 C CNN
F 1 "VCC" H 4765 2623 50  0000 C CNN
F 2 "" H 4750 2450 50  0001 C CNN
F 3 "" H 4750 2450 50  0001 C CNN
	1    4750 2450
	1    0    0    -1  
$EndComp
Wire Wire Line
	4950 2450 4950 2550
Wire Wire Line
	4950 2550 4750 2550
Connection ~ 1700 3900
Wire Wire Line
	2050 3900 1700 3900
$Comp
L Device:R_Small R4
U 1 1 5F8631E9
P 2150 3900
F 0 "R4" V 2230 3900 50  0000 C CNN
F 1 "10kΩ" V 2070 3900 50  0000 C CNN
F 2 "Resistor_SMD:R_0402_1005Metric" H 2150 3900 50  0001 C CNN
F 3 "~" H 2150 3900 50  0001 C CNN
	1    2150 3900
	0    -1   -1   0   
$EndComp
Wire Wire Line
	2050 3750 2350 3750
$Comp
L Device:C_Small C2
U 1 1 5F8536E5
P 2650 4200
F 0 "C2" H 2558 4154 50  0000 R CNN
F 1 "0.01µF" H 2558 4245 50  0000 R CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 2650 4200 50  0001 C CNN
F 3 "~" H 2650 4200 50  0001 C CNN
	1    2650 4200
	-1   0    0    1   
$EndComp
$Comp
L Device:C_Small C1
U 1 1 5F84FD2A
P 2350 4150
F 0 "C1" H 2550 4150 50  0000 R CNN
F 1 "0.01µF" H 2650 4250 50  0000 R CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 2350 4150 50  0001 C CNN
F 3 "~" H 2350 4150 50  0001 C CNN
	1    2350 4150
	-1   0    0    1   
$EndComp
$Comp
L power:PWR_FLAG #FLG0101
U 1 1 5F9565C2
P 4450 2450
F 0 "#FLG0101" H 4450 2525 50  0001 C CNN
F 1 "PWR_FLAG" H 4450 2623 50  0000 C CNN
F 2 "" H 4450 2450 50  0001 C CNN
F 3 "~" H 4450 2450 50  0001 C CNN
	1    4450 2450
	1    0    0    -1  
$EndComp
Wire Wire Line
	3500 4050 3500 4300
Wire Wire Line
	5350 4050 4500 4050
Wire Wire Line
	2350 4050 2350 3750
Connection ~ 2350 3750
Wire Wire Line
	2350 4250 2350 4350
Wire Wire Line
	2650 4350 2650 4300
Wire Wire Line
	2250 3900 2650 3900
Wire Wire Line
	4750 2550 4450 2550
Wire Wire Line
	4450 2450 4450 2550
Connection ~ 4750 2550
Wire Wire Line
	5050 2450 5050 2900
$Comp
L power:PWR_FLAG #FLG0102
U 1 1 5F8C75F0
P 4750 2800
F 0 "#FLG0102" H 4750 2875 50  0001 C CNN
F 1 "PWR_FLAG" H 4750 2973 50  0000 C CNN
F 2 "" H 4750 2800 50  0001 C CNN
F 3 "~" H 4750 2800 50  0001 C CNN
	1    4750 2800
	1    0    0    -1  
$EndComp
Wire Wire Line
	4750 2800 4750 2900
Wire Wire Line
	4750 2900 5050 2900
Connection ~ 5050 2900
Wire Wire Line
	5050 2900 5050 3000
Wire Wire Line
	5150 2450 5150 3750
$Comp
L power:GND #PWR0101
U 1 1 5F8F32F0
P 5050 3000
F 0 "#PWR0101" H 5050 2750 50  0001 C CNN
F 1 "GND" H 5055 2827 50  0000 C CNN
F 2 "" H 5050 3000 50  0001 C CNN
F 3 "" H 5050 3000 50  0001 C CNN
	1    5050 3000
	-1   0    0    -1  
$EndComp
Wire Wire Line
	2650 4100 2650 3900
Connection ~ 2650 3900
Wire Wire Line
	2350 4350 2500 4350
Wire Wire Line
	2500 4350 2500 5100
Connection ~ 2500 4350
Wire Wire Line
	2500 4350 2650 4350
Connection ~ 2500 5100
Wire Wire Line
	2500 5100 1500 5100
$Comp
L Device:R_Small R5
U 1 1 5F90676A
P 3500 3400
F 0 "R5" H 3350 3450 50  0000 L CNN
F 1 "10kΩ" H 3250 3350 50  0000 L CNN
F 2 "Resistor_SMD:R_0402_1005Metric" H 3500 3400 50  0001 C CNN
F 3 "~" H 3500 3400 50  0001 C CNN
	1    3500 3400
	1    0    0    -1  
$EndComp
Wire Wire Line
	3500 3300 3500 3200
$Comp
L Device:R_Small R7
U 1 1 5F906776
P 3700 3400
F 0 "R7" H 3759 3446 50  0000 L CNN
F 1 "10kΩ" H 3759 3355 50  0000 L CNN
F 2 "Resistor_SMD:R_0402_1005Metric" H 3700 3400 50  0001 C CNN
F 3 "~" H 3700 3400 50  0001 C CNN
	1    3700 3400
	1    0    0    -1  
$EndComp
Wire Wire Line
	3700 3300 3700 3200
Wire Wire Line
	3700 3200 3500 3200
Connection ~ 3500 3200
Wire Wire Line
	3500 3200 3500 3100
$Comp
L power:VCC #PWR0104
U 1 1 5F906785
P 3500 3100
F 0 "#PWR0104" H 3500 2950 50  0001 C CNN
F 1 "VCC" H 3515 3273 50  0000 C CNN
F 2 "" H 3500 3100 50  0001 C CNN
F 3 "" H 3500 3100 50  0001 C CNN
	1    3500 3100
	1    0    0    -1  
$EndComp
Wire Wire Line
	3500 3500 3500 4050
Connection ~ 3500 4050
Wire Wire Line
	3700 3500 3700 4200
Connection ~ 3700 4200
$Comp
L Device:R_Small R6
U 1 1 5F913015
P 3950 4050
F 0 "R6" V 3870 4050 50  0000 C CNN
F 1 "10kΩ" V 4020 4050 50  0000 C CNN
F 2 "Resistor_SMD:R_0402_1005Metric" H 3950 4050 50  0001 C CNN
F 3 "~" H 3950 4050 50  0001 C CNN
	1    3950 4050
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R8
U 1 1 5F913020
P 4250 4200
F 0 "R8" V 4330 4200 50  0000 C CNN
F 1 "10kΩ" V 4170 4200 50  0000 C CNN
F 2 "Resistor_SMD:R_0402_1005Metric" H 4250 4200 50  0001 C CNN
F 3 "~" H 4250 4200 50  0001 C CNN
	1    4250 4200
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3850 4050 3500 4050
Wire Wire Line
	4350 4200 4800 4200
Wire Wire Line
	2350 3750 5150 3750
Wire Wire Line
	2650 3900 5250 3900
$Comp
L Device:C_Small C4
U 1 1 5F923DFB
P 4800 4400
F 0 "C4" H 4700 4400 50  0000 R CNN
F 1 "0.01µF" H 4750 4500 50  0000 R CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 4800 4400 50  0001 C CNN
F 3 "~" H 4800 4400 50  0001 C CNN
	1    4800 4400
	-1   0    0    1   
$EndComp
$Comp
L Device:C_Small C3
U 1 1 5F923E05
P 4500 4400
F 0 "C3" H 4700 4400 50  0000 R CNN
F 1 "0.01µF" H 4800 4500 50  0000 R CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 4500 4400 50  0001 C CNN
F 3 "~" H 4500 4400 50  0001 C CNN
	1    4500 4400
	-1   0    0    1   
$EndComp
Wire Wire Line
	4500 4500 4500 4550
Wire Wire Line
	4800 4550 4800 4500
Wire Wire Line
	4500 4550 4650 4550
Wire Wire Line
	4650 4550 4650 5100
Connection ~ 4650 4550
Wire Wire Line
	4650 4550 4800 4550
Wire Wire Line
	4500 4300 4500 4050
Connection ~ 4500 4050
Wire Wire Line
	4500 4050 4050 4050
Wire Wire Line
	4800 4300 4800 4200
Connection ~ 4800 4200
Wire Wire Line
	4800 4200 5450 4200
Wire Wire Line
	3500 5100 4650 5100
Connection ~ 3500 5100
$EndSCHEMATC
