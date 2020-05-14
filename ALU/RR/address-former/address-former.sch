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
L triador:trimux U1
U 1 1 5EC180D8
P 2100 2500
F 0 "U1" H 2075 2152 50  0000 C CNN
F 1 "trimux" H 2075 2061 50  0000 C CNN
F 2 "ALU:trimux1x15_2.54" H 2400 2875 50  0001 C CNN
F 3 "" H 2400 2875 50  0001 C CNN
	1    2100 2500
	1    0    0    -1  
$EndComp
$Comp
L triador:trimux U1
U 2 1 5EC180DE
P 3550 2500
F 0 "U1" H 3525 2152 50  0000 C CNN
F 1 "trimux" H 3525 2061 50  0000 C CNN
F 2 "ALU:trimux1x15_2.54" H 3850 2875 50  0001 C CNN
F 3 "" H 3850 2875 50  0001 C CNN
	2    3550 2500
	1    0    0    -1  
$EndComp
$Comp
L triador:trimux U1
U 3 1 5EC180E4
P 1850 950
F 0 "U1" H 1681 1315 50  0000 C CNN
F 1 "trimux" H 1681 1224 50  0000 C CNN
F 2 "ALU:trimux1x15_2.54" H 2150 1325 50  0001 C CNN
F 3 "" H 2150 1325 50  0001 C CNN
	3    1850 950 
	0    -1   1    0   
$EndComp
$Comp
L triador:trimux U2
U 1 1 5EC180EA
P 1700 3850
F 0 "U2" H 1675 3502 50  0000 C CNN
F 1 "trimux" H 1675 3411 50  0000 C CNN
F 2 "ALU:trimux1x15_2.54" H 2000 4225 50  0001 C CNN
F 3 "" H 2000 4225 50  0001 C CNN
	1    1700 3850
	1    0    0    -1  
$EndComp
$Comp
L triador:trimux U2
U 2 1 5EC180F0
P 2300 4950
F 0 "U2" H 2275 4602 50  0000 C CNN
F 1 "trimux" H 2275 4511 50  0000 C CNN
F 2 "ALU:trimux1x15_2.54" H 2600 5325 50  0001 C CNN
F 3 "" H 2600 5325 50  0001 C CNN
	2    2300 4950
	1    0    0    -1  
$EndComp
$Comp
L triador:trimux U2
U 3 1 5EC180F6
P 2500 950
F 0 "U2" H 2331 1315 50  0000 C CNN
F 1 "trimux" H 2331 1224 50  0000 C CNN
F 2 "ALU:trimux1x15_2.54" H 2800 1325 50  0001 C CNN
F 3 "" H 2800 1325 50  0001 C CNN
	3    2500 950 
	0    -1   1    0   
$EndComp
$Comp
L triador:trimux U3
U 1 1 5EC180FC
P 3000 6000
F 0 "U3" H 2975 5652 50  0000 C CNN
F 1 "trimux" H 2975 5561 50  0000 C CNN
F 2 "ALU:trimux1x15_2.54" H 3300 6375 50  0001 C CNN
F 3 "" H 3300 6375 50  0001 C CNN
	1    3000 6000
	1    0    0    -1  
$EndComp
$Comp
L triador:trimux U3
U 2 1 5EC18102
P 4900 3000
F 0 "U3" H 4875 2652 50  0000 C CNN
F 1 "trimux" H 4875 2561 50  0000 C CNN
F 2 "ALU:trimux1x15_2.54" H 5200 3375 50  0001 C CNN
F 3 "" H 5200 3375 50  0001 C CNN
	2    4900 3000
	1    0    0    -1  
$EndComp
$Comp
L triador:trimux U3
U 3 1 5EC18108
P 3150 950
F 0 "U3" H 2981 1315 50  0000 C CNN
F 1 "trimux" H 2981 1224 50  0000 C CNN
F 2 "ALU:trimux1x15_2.54" H 3450 1325 50  0001 C CNN
F 3 "" H 3450 1325 50  0001 C CNN
	3    3150 950 
	0    -1   1    0   
$EndComp
Wire Wire Line
	625  1100 625  1250
Text Label 1250 1000 0    50   ~ 0
+5V
Text Label 625  1100 0    50   ~ 0
-5V
Text Label 950  1050 0    50   ~ 0
GND
$Comp
L Device:R_Small R1
U 1 1 5EC18112
P 1600 2650
F 0 "R1" V 1530 2650 50  0000 C CNN
F 1 "150" V 1600 2650 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" H 1600 2650 50  0001 C CNN
F 3 "~" H 1600 2650 50  0001 C CNN
	1    1600 2650
	0    1    1    0   
$EndComp
Wire Wire Line
	1850 2650 1700 2650
Wire Wire Line
	1700 2350 1850 2350
Wire Wire Line
	3150 2350 3300 2350
Wire Wire Line
	1200 2650 1500 2650
Text Label 1200 2650 0    50   ~ 0
+5V
Wire Wire Line
	1200 2350 1500 2350
Wire Wire Line
	2650 2350 2950 2350
Text Label 1200 2350 0    50   ~ 0
-5V
Text Label 2650 2350 0    50   ~ 0
-5V
Wire Wire Line
	2750 5850 2600 5850
Wire Wire Line
	2600 6150 2750 6150
Wire Wire Line
	3150 2650 3300 2650
Wire Wire Line
	2650 2650 2950 2650
Text Label 2100 5850 0    50   ~ 0
+5V
Wire Wire Line
	2100 5850 2400 5850
Wire Wire Line
	2100 6150 2400 6150
Text Label 2100 6150 0    50   ~ 0
-5V
Text Label 2650 2650 0    50   ~ 0
+5V
Wire Wire Line
	2050 5100 1900 5100
Wire Wire Line
	1300 3850 1450 3850
Wire Wire Line
	1400 5100 1700 5100
Wire Wire Line
	800  3850 1100 3850
Text Label 1400 5100 0    50   ~ 0
-5V
Wire Wire Line
	2050 4950 1900 4950
Wire Wire Line
	2600 6000 2750 6000
Wire Wire Line
	1900 4800 2050 4800
Wire Wire Line
	1400 4950 1700 4950
Text Label 1400 4800 0    50   ~ 0
+5V
Wire Wire Line
	2100 6000 2400 6000
Wire Wire Line
	1400 4800 1700 4800
Text Label 2100 6000 0    50   ~ 0
GND
Text Label 1400 4950 0    50   ~ 0
GND
Text Label 800  3850 0    50   ~ 0
GND
$Comp
L triador:+5V #PWR03
U 1 1 5EC1813C
P 1250 900
F 0 "#PWR03" H 1250 750 50  0001 C CNN
F 1 "+5V" H 1265 1073 50  0000 C CNN
F 2 "" H 1250 900 50  0001 C CNN
F 3 "" H 1250 900 50  0001 C CNN
	1    1250 900 
	1    0    0    -1  
$EndComp
$Comp
L triador:-5V #PWR01
U 1 1 5EC18142
P 625 1250
F 0 "#PWR01" H 625 1350 50  0001 C CNN
F 1 "-5V" H 640 1423 50  0000 C CNN
F 2 "" H 625 1250 50  0001 C CNN
F 3 "" H 625 1250 50  0001 C CNN
	1    625  1250
	-1   0    0    1   
$EndComp
$Comp
L triador:GNDREF #PWR02
U 1 1 5EC18148
P 950 1175
F 0 "#PWR02" H 950 925 50  0001 C CNN
F 1 "GNDREF" H 955 1002 50  0000 C CNN
F 2 "" H 950 1175 50  0001 C CNN
F 3 "" H 950 1175 50  0001 C CNN
	1    950  1175
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R2
U 1 1 5EC18177
P 1600 2350
F 0 "R2" V 1530 2350 50  0000 C CNN
F 1 "150" V 1600 2350 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" H 1600 2350 50  0001 C CNN
F 3 "~" H 1600 2350 50  0001 C CNN
	1    1600 2350
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R3
U 1 1 5EC1817D
P 3050 2350
F 0 "R3" V 2980 2350 50  0000 C CNN
F 1 "150" V 3050 2350 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" H 3050 2350 50  0001 C CNN
F 3 "~" H 3050 2350 50  0001 C CNN
	1    3050 2350
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R4
U 1 1 5EC18183
P 3050 2650
F 0 "R4" V 2980 2650 50  0000 C CNN
F 1 "150" V 3050 2650 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" H 3050 2650 50  0001 C CNN
F 3 "~" H 3050 2650 50  0001 C CNN
	1    3050 2650
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R5
U 1 1 5EC18189
P 2500 5850
F 0 "R5" V 2430 5850 50  0000 C CNN
F 1 "150" V 2500 5850 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" H 2500 5850 50  0001 C CNN
F 3 "~" H 2500 5850 50  0001 C CNN
	1    2500 5850
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R6
U 1 1 5EC1818F
P 2500 6150
F 0 "R6" V 2430 6150 50  0000 C CNN
F 1 "150" V 2500 6150 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" H 2500 6150 50  0001 C CNN
F 3 "~" H 2500 6150 50  0001 C CNN
	1    2500 6150
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R7
U 1 1 5EC18195
P 1800 5100
F 0 "R7" V 1730 5100 50  0000 C CNN
F 1 "150" V 1800 5100 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" H 1800 5100 50  0001 C CNN
F 3 "~" H 1800 5100 50  0001 C CNN
	1    1800 5100
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R8
U 1 1 5EC1819B
P 1200 3850
F 0 "R8" V 1130 3850 50  0000 C CNN
F 1 "150" V 1200 3850 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" H 1200 3850 50  0001 C CNN
F 3 "~" H 1200 3850 50  0001 C CNN
	1    1200 3850
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R10
U 1 1 5EC181A7
P 1800 4950
F 0 "R10" V 1730 4950 50  0000 C CNN
F 1 "150" V 1800 4950 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" H 1800 4950 50  0001 C CNN
F 3 "~" H 1800 4950 50  0001 C CNN
	1    1800 4950
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R11
U 1 1 5EC181AD
P 2500 6000
F 0 "R11" V 2430 6000 50  0000 C CNN
F 1 "150" V 2500 6000 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" H 2500 6000 50  0001 C CNN
F 3 "~" H 2500 6000 50  0001 C CNN
	1    2500 6000
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R12
U 1 1 5EC181B3
P 1800 4800
F 0 "R12" V 1730 4800 50  0000 C CNN
F 1 "150" V 1800 4800 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" H 1800 4800 50  0001 C CNN
F 3 "~" H 1800 4800 50  0001 C CNN
	1    1800 4800
	0    1    1    0   
$EndComp
Wire Wire Line
	1950 1000 1950 950 
Wire Wire Line
	1950 1000 2600 1000
Wire Wire Line
	2600 1000 2600 950 
Connection ~ 1950 1000
Wire Wire Line
	2600 1000 3250 1000
Wire Wire Line
	3250 1000 3250 950 
Connection ~ 2600 1000
Wire Wire Line
	1250 900  1250 1000
Wire Wire Line
	1250 1000 1950 1000
Wire Wire Line
	950  1050 1850 1050
Wire Wire Line
	1850 1050 1850 950 
Wire Wire Line
	950  1050 950  1175
Wire Wire Line
	1850 1050 2500 1050
Wire Wire Line
	2500 1050 2500 950 
Connection ~ 1850 1050
Wire Wire Line
	2500 1050 3150 1050
Wire Wire Line
	3150 1050 3150 950 
Connection ~ 2500 1050
Wire Wire Line
	1750 1100 1750 950 
Wire Wire Line
	1750 1100 2400 1100
Wire Wire Line
	2400 1100 2400 950 
Connection ~ 1750 1100
Wire Wire Line
	2400 1100 3050 1100
Wire Wire Line
	3050 1100 3050 950 
Connection ~ 2400 1100
Wire Wire Line
	625  1100 1750 1100
Wire Wire Line
	1050 6850 1050 6750
Wire Wire Line
	1050 7350 1050 7600
Wire Wire Line
	1050 7600 650  7600
Text Label 750  7600 0    50   ~ 0
+5V
Text Label 850  7750 0    50   ~ 0
-5V
Wire Wire Line
	1450 6850 1450 6750
Wire Wire Line
	1450 6750 1750 6750
Wire Wire Line
	1350 6850 1350 6600
Wire Wire Line
	1350 6600 1750 6600
Wire Wire Line
	1450 7350 1450 7450
Wire Wire Line
	1450 7450 1750 7450
Text Label 1550 7450 0    50   ~ 0
OC2
Text Label 1550 6600 0    50   ~ 0
OC3
Wire Wire Line
	1350 7350 1350 7600
Wire Wire Line
	1350 7600 1600 7600
Wire Wire Line
	1250 7350 1250 7750
Wire Wire Line
	1250 7750 1500 7750
Wire Wire Line
	1250 6850 1250 6450
Wire Wire Line
	1250 6450 1650 6450
Text Label 1450 7600 0    50   ~ 0
Rlo
Text Label 1450 6450 0    50   ~ 0
Rmi
Text Label 1350 7750 0    50   ~ 0
Rhi
Wire Wire Line
	1150 6850 1150 6600
Wire Wire Line
	1150 6600 850  6600
Wire Wire Line
	1150 7350 1150 7750
Wire Wire Line
	1150 7750 800  7750
Text Label 950  6600 0    50   ~ 0
CYCLE
$Comp
L triador:trimux U4
U 1 1 5EC18201
P 6500 3000
F 0 "U4" H 6475 2652 50  0000 C CNN
F 1 "trimux" H 6475 2561 50  0000 C CNN
F 2 "ALU:trimux1x15_2.54" H 6800 3375 50  0001 C CNN
F 3 "" H 6800 3375 50  0001 C CNN
	1    6500 3000
	1    0    0    -1  
$EndComp
$Comp
L triador:trimux U4
U 2 1 5EC18207
P 8150 3000
F 0 "U4" H 8125 2652 50  0000 C CNN
F 1 "trimux" H 8125 2561 50  0000 C CNN
F 2 "ALU:trimux1x15_2.54" H 8450 3375 50  0001 C CNN
F 3 "" H 8450 3375 50  0001 C CNN
	2    8150 3000
	1    0    0    -1  
$EndComp
$Comp
L triador:trimux U5
U 1 1 5EC1820D
P 4900 4150
F 0 "U5" H 4875 3802 50  0000 C CNN
F 1 "trimux" H 4875 3711 50  0000 C CNN
F 2 "ALU:trimux1x15_2.54" H 5200 4525 50  0001 C CNN
F 3 "" H 5200 4525 50  0001 C CNN
	1    4900 4150
	1    0    0    -1  
$EndComp
$Comp
L triador:trimux U5
U 2 1 5EC18213
P 6500 4150
F 0 "U5" H 6475 3802 50  0000 C CNN
F 1 "trimux" H 6475 3711 50  0000 C CNN
F 2 "ALU:trimux1x15_2.54" H 6800 4525 50  0001 C CNN
F 3 "" H 6800 4525 50  0001 C CNN
	2    6500 4150
	1    0    0    -1  
$EndComp
$Comp
L triador:trimux U6
U 1 1 5EC18219
P 8150 4150
F 0 "U6" H 8125 3802 50  0000 C CNN
F 1 "trimux" H 8125 3711 50  0000 C CNN
F 2 "ALU:trimux1x15_2.54" H 8450 4525 50  0001 C CNN
F 3 "" H 8450 4525 50  0001 C CNN
	1    8150 4150
	1    0    0    -1  
$EndComp
$Comp
L triador:trimux U6
U 2 1 5EC1821F
P 5300 6000
F 0 "U6" H 5275 5652 50  0000 C CNN
F 1 "trimux" H 5275 5561 50  0000 C CNN
F 2 "ALU:trimux1x15_2.54" H 5600 6375 50  0001 C CNN
F 3 "" H 5600 6375 50  0001 C CNN
	2    5300 6000
	1    0    0    -1  
$EndComp
Wire Wire Line
	4650 2850 4500 2850
Wire Wire Line
	1300 4000 1450 4000
Text Label 4000 2850 0    50   ~ 0
+5V
Wire Wire Line
	4000 2850 4300 2850
Wire Wire Line
	800  4000 1100 4000
Text Label 800  4000 0    50   ~ 0
-5V
$Comp
L Device:R_Small R18
U 1 1 5EC18288
P 4400 2850
F 0 "R18" V 4330 2850 50  0000 C CNN
F 1 "150" V 4400 2850 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" H 4400 2850 50  0001 C CNN
F 3 "~" H 4400 2850 50  0001 C CNN
	1    4400 2850
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R19
U 1 1 5EC1828E
P 1200 4000
F 0 "R19" V 1130 4000 50  0000 C CNN
F 1 "150" V 1200 4000 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" H 1200 4000 50  0001 C CNN
F 3 "~" H 1200 4000 50  0001 C CNN
	1    1200 4000
	0    1    1    0   
$EndComp
$Comp
L triador:trimux U7
U 1 1 5EC182BE
P 6900 6000
F 0 "U7" H 6875 5652 50  0000 C CNN
F 1 "trimux" H 6875 5561 50  0000 C CNN
F 2 "ALU:trimux1x15_2.54" H 7200 6375 50  0001 C CNN
F 3 "" H 7200 6375 50  0001 C CNN
	1    6900 6000
	1    0    0    -1  
$EndComp
$Comp
L triador:trimux U7
U 2 1 5EC182C4
P 8550 6000
F 0 "U7" H 8525 5652 50  0000 C CNN
F 1 "trimux" H 8525 5561 50  0000 C CNN
F 2 "ALU:trimux1x15_2.54" H 8850 6375 50  0001 C CNN
F 3 "" H 8850 6375 50  0001 C CNN
	2    8550 6000
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R27
U 1 1 5EC182E2
P 1200 3700
F 0 "R27" V 1130 3700 50  0000 C CNN
F 1 "150" V 1200 3700 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" H 1200 3700 50  0001 C CNN
F 3 "~" H 1200 3700 50  0001 C CNN
	1    1200 3700
	0    1    1    0   
$EndComp
Wire Wire Line
	1450 3700 1300 3700
Wire Wire Line
	800  3700 1100 3700
Text Label 800  3700 0    50   ~ 0
+5V
$Comp
L triador:trimux U7
U 3 1 5EC1837F
P 5750 950
F 0 "U7" H 5581 1315 50  0000 C CNN
F 1 "trimux" H 5581 1224 50  0000 C CNN
F 2 "ALU:trimux1x15_2.54" H 6050 1325 50  0001 C CNN
F 3 "" H 6050 1325 50  0001 C CNN
	3    5750 950 
	0    -1   1    0   
$EndComp
$Comp
L triador:trimux U4
U 3 1 5EC18385
P 3800 950
F 0 "U4" H 3631 1315 50  0000 C CNN
F 1 "trimux" H 3631 1224 50  0000 C CNN
F 2 "ALU:trimux1x15_2.54" H 4100 1325 50  0001 C CNN
F 3 "" H 4100 1325 50  0001 C CNN
	3    3800 950 
	0    -1   1    0   
$EndComp
$Comp
L triador:trimux U5
U 3 1 5EC18391
P 4450 950
F 0 "U5" H 4281 1315 50  0000 C CNN
F 1 "trimux" H 4281 1224 50  0000 C CNN
F 2 "ALU:trimux1x15_2.54" H 4750 1325 50  0001 C CNN
F 3 "" H 4750 1325 50  0001 C CNN
	3    4450 950 
	0    -1   1    0   
$EndComp
$Comp
L triador:trimux U6
U 3 1 5EC1839D
P 5100 950
F 0 "U6" H 4931 1315 50  0000 C CNN
F 1 "trimux" H 4931 1224 50  0000 C CNN
F 2 "ALU:trimux1x15_2.54" H 5400 1325 50  0001 C CNN
F 3 "" H 5400 1325 50  0001 C CNN
	3    5100 950 
	0    -1   1    0   
$EndComp
Wire Wire Line
	3250 1000 3900 1000
Wire Wire Line
	3900 1000 3900 950 
Connection ~ 3250 1000
Wire Wire Line
	3900 1000 4550 1000
Wire Wire Line
	4550 1000 4550 950 
Connection ~ 3900 1000
Wire Wire Line
	4550 1000 5200 1000
Wire Wire Line
	5200 1000 5200 950 
Connection ~ 4550 1000
Wire Wire Line
	5200 1000 5850 1000
Wire Wire Line
	5850 1000 5850 950 
Connection ~ 5200 1000
Connection ~ 5850 1000
Wire Wire Line
	3150 1050 3800 1050
Wire Wire Line
	3800 1050 3800 950 
Connection ~ 3150 1050
Wire Wire Line
	3800 1050 4450 1050
Wire Wire Line
	4450 1050 4450 950 
Connection ~ 3800 1050
Wire Wire Line
	4450 1050 5100 1050
Wire Wire Line
	5100 1050 5100 950 
Connection ~ 4450 1050
Wire Wire Line
	5100 1050 5750 1050
Wire Wire Line
	5750 1050 5750 950 
Connection ~ 5100 1050
Connection ~ 5750 1050
Wire Wire Line
	3050 1100 3700 1100
Wire Wire Line
	3700 1100 3700 950 
Connection ~ 3050 1100
Wire Wire Line
	3700 1100 4350 1100
Wire Wire Line
	4350 1100 4350 950 
Connection ~ 3700 1100
Wire Wire Line
	4350 1100 5000 1100
Wire Wire Line
	5000 1100 5000 950 
Connection ~ 4350 1100
Wire Wire Line
	5000 1100 5650 1100
Wire Wire Line
	5650 1100 5650 950 
Connection ~ 5000 1100
Connection ~ 5650 1100
$Comp
L Device:CP C13
U 1 1 5EC183D9
P 10100 900
F 0 "C13" H 10218 946 50  0000 L CNN
F 1 "47u" H 10218 855 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D5.0mm_P2.50mm" H 10138 750 50  0001 C CNN
F 3 "~" H 10100 900 50  0001 C CNN
	1    10100 900 
	1    0    0    -1  
$EndComp
$Comp
L Device:CP C14
U 1 1 5EC183DF
P 10150 1550
F 0 "C14" H 10268 1596 50  0000 L CNN
F 1 "47u" H 10268 1505 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D5.0mm_P2.50mm" H 10188 1400 50  0001 C CNN
F 3 "~" H 10150 1550 50  0001 C CNN
	1    10150 1550
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C3
U 1 1 5EC183E5
P 9050 950
F 0 "C3" H 9000 1150 50  0000 L CNN
F 1 "100n" H 9000 750 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 9050 950 50  0001 C CNN
F 3 "~" H 9050 950 50  0001 C CNN
	1    9050 950 
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C5
U 1 1 5EC183EB
P 9250 950
F 0 "C5" H 9200 1150 50  0000 L CNN
F 1 "100n" H 9200 750 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 9250 950 50  0001 C CNN
F 3 "~" H 9250 950 50  0001 C CNN
	1    9250 950 
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C7
U 1 1 5EC183F1
P 9450 950
F 0 "C7" H 9400 1150 50  0000 L CNN
F 1 "100n" H 9400 750 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 9450 950 50  0001 C CNN
F 3 "~" H 9450 950 50  0001 C CNN
	1    9450 950 
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C4
U 1 1 5EC18403
P 9050 1500
F 0 "C4" H 9000 1700 50  0000 L CNN
F 1 "100n" H 9000 1300 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 9050 1500 50  0001 C CNN
F 3 "~" H 9050 1500 50  0001 C CNN
	1    9050 1500
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C6
U 1 1 5EC18409
P 9250 1500
F 0 "C6" H 9200 1700 50  0000 L CNN
F 1 "100n" H 9200 1300 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 9250 1500 50  0001 C CNN
F 3 "~" H 9250 1500 50  0001 C CNN
	1    9250 1500
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C8
U 1 1 5EC1840F
P 9450 1500
F 0 "C8" H 9400 1700 50  0000 L CNN
F 1 "100n" H 9400 1300 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 9450 1500 50  0001 C CNN
F 3 "~" H 9450 1500 50  0001 C CNN
	1    9450 1500
	1    0    0    -1  
$EndComp
Wire Wire Line
	9450 1600 9450 1650
Connection ~ 9450 1650
Wire Wire Line
	9250 1600 9250 1650
Connection ~ 9250 1650
Wire Wire Line
	9250 1650 9450 1650
Wire Wire Line
	9050 1600 9050 1650
Connection ~ 9050 1650
Wire Wire Line
	9050 1650 9250 1650
Wire Wire Line
	9050 1400 9050 1350
Connection ~ 9050 1350
Wire Wire Line
	9250 1400 9250 1350
Connection ~ 9250 1350
Wire Wire Line
	9250 1350 9050 1350
Wire Wire Line
	9450 1400 9450 1350
Connection ~ 9450 1350
Wire Wire Line
	9450 1350 9250 1350
Wire Wire Line
	9050 1050 9050 1100
Wire Wire Line
	9250 1050 9250 1100
Connection ~ 9250 1100
Wire Wire Line
	9250 1100 9050 1100
Wire Wire Line
	9450 1050 9450 1100
Connection ~ 9450 1100
Wire Wire Line
	9450 1100 9250 1100
Wire Wire Line
	9450 850  9450 800 
Connection ~ 9450 800 
Wire Wire Line
	9250 800  9250 850 
Connection ~ 9250 800 
Wire Wire Line
	9250 800  9450 800 
Wire Wire Line
	9050 850  9050 800 
Connection ~ 9050 800 
Wire Wire Line
	9050 800  9250 800 
Wire Wire Line
	10150 1350 10150 1400
Wire Wire Line
	10050 1650 10050 1750
Wire Wire Line
	10050 1750 10150 1750
Wire Wire Line
	10150 1750 10150 1700
Wire Wire Line
	10100 1100 10100 1050
Wire Wire Line
	10000 800  10000 700 
Wire Wire Line
	10000 700  10100 700 
Wire Wire Line
	10100 700  10100 750 
$Comp
L Device:LED_Dual_2pin D1
U 1 1 5EC18464
P 10300 2600
F 0 "D1" H 10300 2996 50  0000 C CNN
F 1 "LED_Dual_2pin" H 10300 2905 50  0000 C CNN
F 2 "LED_THT:LED_D5.0mm" H 10300 2600 50  0001 C CNN
F 3 "~" H 10300 2600 50  0001 C CNN
	1    10300 2600
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R33
U 1 1 5EC1846A
P 10750 2800
F 0 "R33" V 10680 2800 50  0000 C CNN
F 1 "330" V 10750 2800 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" H 10750 2800 50  0001 C CNN
F 3 "~" H 10750 2800 50  0001 C CNN
	1    10750 2800
	-1   0    0    1   
$EndComp
Wire Wire Line
	10600 2600 10750 2600
Wire Wire Line
	10750 2600 10750 2700
Wire Wire Line
	10750 2900 10750 2950
Wire Wire Line
	9750 2600 9900 2600
Wire Wire Line
	9900 2600 9900 2950
Wire Wire Line
	9900 2950 9100 2950
Wire Wire Line
	9100 2950 9100 2700
Wire Wire Line
	9100 2700 9150 2700
Connection ~ 9900 2600
Wire Wire Line
	9900 2600 10000 2600
Wire Wire Line
	9150 2500 8950 2500
Wire Wire Line
	10100 1100 10150 1100
Connection ~ 10100 1100
Connection ~ 10150 1350
Text Label 8950 2500 0    50   ~ 0
Rlo
$Comp
L Device:LED_Dual_2pin D2
U 1 1 5EC1847F
P 10300 3650
F 0 "D2" H 10300 4046 50  0000 C CNN
F 1 "LED_Dual_2pin" H 10300 3955 50  0000 C CNN
F 2 "LED_THT:LED_D5.0mm" H 10300 3650 50  0001 C CNN
F 3 "~" H 10300 3650 50  0001 C CNN
	1    10300 3650
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R34
U 1 1 5EC18485
P 10750 3850
F 0 "R34" V 10680 3850 50  0000 C CNN
F 1 "330" V 10750 3850 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" H 10750 3850 50  0001 C CNN
F 3 "~" H 10750 3850 50  0001 C CNN
	1    10750 3850
	-1   0    0    1   
$EndComp
Wire Wire Line
	10600 3650 10750 3650
Wire Wire Line
	10750 3650 10750 3750
Wire Wire Line
	10750 3950 10750 4000
Wire Wire Line
	9750 3650 9900 3650
Wire Wire Line
	9900 3650 9900 4000
Wire Wire Line
	9900 4000 9100 4000
Wire Wire Line
	9100 4000 9100 3750
Wire Wire Line
	9100 3750 9150 3750
Connection ~ 9900 3650
Wire Wire Line
	9900 3650 10000 3650
Wire Wire Line
	9150 3550 8950 3550
Text Label 8950 3550 0    50   ~ 0
Rmi
$Comp
L Device:LED_Dual_2pin D3
U 1 1 5EC18497
P 10300 4700
F 0 "D3" H 10300 5096 50  0000 C CNN
F 1 "LED_Dual_2pin" H 10300 5005 50  0000 C CNN
F 2 "LED_THT:LED_D5.0mm" H 10300 4700 50  0001 C CNN
F 3 "~" H 10300 4700 50  0001 C CNN
	1    10300 4700
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R35
U 1 1 5EC1849D
P 10750 4900
F 0 "R35" V 10680 4900 50  0000 C CNN
F 1 "330" V 10750 4900 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" H 10750 4900 50  0001 C CNN
F 3 "~" H 10750 4900 50  0001 C CNN
	1    10750 4900
	-1   0    0    1   
$EndComp
Wire Wire Line
	10600 4700 10750 4700
Wire Wire Line
	10750 4700 10750 4800
Wire Wire Line
	10750 5000 10750 5050
Wire Wire Line
	9750 4700 9900 4700
Wire Wire Line
	9900 4700 9900 5050
Wire Wire Line
	9900 5050 9100 5050
Wire Wire Line
	9100 5050 9100 4800
Wire Wire Line
	9100 4800 9150 4800
Connection ~ 9900 4700
Wire Wire Line
	9900 4700 10000 4700
Wire Wire Line
	9150 4600 8950 4600
Text Label 8950 4600 0    50   ~ 0
Rhi
$Comp
L Amplifier_Operational:LM324 U8
U 1 1 5EC184C7
P 9450 2600
F 0 "U8" H 9450 2967 50  0000 C CNN
F 1 "LM324" H 9450 2876 50  0000 C CNN
F 2 "Package_SO:SOIC-14_3.9x8.7mm_P1.27mm" H 9400 2700 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/lm2902-n.pdf" H 9500 2800 50  0001 C CNN
	1    9450 2600
	1    0    0    -1  
$EndComp
$Comp
L Amplifier_Operational:LM324 U8
U 2 1 5EC184CD
P 9450 3650
F 0 "U8" H 9450 4017 50  0000 C CNN
F 1 "LM324" H 9450 3926 50  0000 C CNN
F 2 "Package_SO:SOIC-14_3.9x8.7mm_P1.27mm" H 9400 3750 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/lm2902-n.pdf" H 9500 3850 50  0001 C CNN
	2    9450 3650
	1    0    0    -1  
$EndComp
$Comp
L Amplifier_Operational:LM324 U8
U 3 1 5EC184D3
P 9450 4700
F 0 "U8" H 9450 5067 50  0000 C CNN
F 1 "LM324" H 9450 4976 50  0000 C CNN
F 2 "Package_SO:SOIC-14_3.9x8.7mm_P1.27mm" H 9400 4800 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/lm2902-n.pdf" H 9500 4900 50  0001 C CNN
	3    9450 4700
	1    0    0    -1  
$EndComp
$Comp
L Amplifier_Operational:LM324 U8
U 5 1 5EC184DF
P 10900 1300
F 0 "U8" H 10858 1346 50  0000 L CNN
F 1 "LM324" H 10858 1255 50  0000 L CNN
F 2 "Package_SO:SOIC-14_3.9x8.7mm_P1.27mm" H 10850 1400 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/lm2902-n.pdf" H 10950 1500 50  0001 C CNN
	5    10900 1300
	1    0    0    -1  
$EndComp
Wire Wire Line
	10150 1100 10150 1350
Wire Wire Line
	7550 1000 7550 800 
Wire Wire Line
	7950 1050 7950 1100
Wire Wire Line
	7950 1100 7950 1350
Connection ~ 7950 1100
Wire Wire Line
	7900 1100 7900 1400
Wire Wire Line
	7900 1400 8100 1400
Wire Wire Line
	8100 1400 8100 1650
Wire Wire Line
	10100 700  10800 700 
Wire Wire Line
	10800 700  10800 1000
Connection ~ 10100 700 
Wire Wire Line
	10800 1750 10150 1750
Wire Wire Line
	10800 1600 10800 1750
Connection ~ 10150 1750
Wire Wire Line
	7550 800  8850 800 
Connection ~ 9050 1100
Wire Wire Line
	7950 1100 8850 1100
Wire Wire Line
	7950 1350 8850 1350
Wire Wire Line
	8100 1650 8850 1650
Wire Wire Line
	10150 1350 10600 1350
Wire Wire Line
	10600 1350 10600 2000
Wire Wire Line
	10600 2000 11050 2000
Wire Wire Line
	11050 2000 11050 2950
Wire Wire Line
	11050 2950 10750 2950
Wire Wire Line
	11050 2950 11050 4000
Wire Wire Line
	11050 4000 10750 4000
Connection ~ 11050 2950
Wire Wire Line
	11050 4000 11050 5050
Wire Wire Line
	11050 5050 10750 5050
Connection ~ 11050 4000
$Comp
L Device:C_Small C1
U 1 1 5EC1850C
P 8850 950
F 0 "C1" H 8800 1150 50  0000 L CNN
F 1 "100n" H 8800 750 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 8850 950 50  0001 C CNN
F 3 "~" H 8850 950 50  0001 C CNN
	1    8850 950 
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C2
U 1 1 5EC18512
P 8850 1500
F 0 "C2" H 8800 1700 50  0000 L CNN
F 1 "100n" H 8800 1300 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 8850 1500 50  0001 C CNN
F 3 "~" H 8850 1500 50  0001 C CNN
	1    8850 1500
	1    0    0    -1  
$EndComp
Wire Wire Line
	8850 800  8850 850 
Connection ~ 8850 800 
Wire Wire Line
	8850 800  9050 800 
Wire Wire Line
	8850 1050 8850 1100
Connection ~ 8850 1100
Wire Wire Line
	8850 1100 9050 1100
Wire Wire Line
	8850 1350 8850 1400
Connection ~ 8850 1350
Wire Wire Line
	8850 1350 9050 1350
Wire Wire Line
	8850 1600 8850 1650
Connection ~ 8850 1650
Wire Wire Line
	8850 1650 9050 1650
$Comp
L Connector_Generic:Conn_02x05_Odd_Even J1
U 1 1 5EC0BC7D
P 1250 7050
F 0 "J1" V 1254 7330 50  0000 L CNN
F 1 "Conn_02x05_Odd_Even" V 1345 7330 50  0000 L CNN
F 2 "Connector_IDC:IDC-Header_2x05_P2.54mm_Vertical" H 1250 7050 50  0001 C CNN
F 3 "~" H 1250 7050 50  0001 C CNN
	1    1250 7050
	0    1    1    0   
$EndComp
Text Label 700  6750 0    50   ~ 0
GND
Wire Wire Line
	1050 6750 650  6750
Wire Wire Line
	2300 2500 3300 2500
Wire Wire Line
	2100 2200 2100 1750
Wire Wire Line
	3550 2200 3550 1750
Wire Wire Line
	1850 2500 950  2500
Wire Wire Line
	950  2500 950  1750
Text Label 950  1750 0    50   ~ 0
OC1
Text Label 2100 1750 0    50   ~ 0
OC2
Text Label 3550 1750 0    50   ~ 0
OC3
Wire Wire Line
	1700 3550 1700 3350
Wire Wire Line
	2300 4650 2300 4450
Wire Wire Line
	3000 5700 3000 5500
Text Label 1700 3350 0    50   ~ 0
OC1
Text Label 2300 4450 0    50   ~ 0
OC2
Text Label 3000 5500 0    50   ~ 0
OC3
Wire Wire Line
	1900 3850 2000 3850
Wire Wire Line
	2500 4950 2600 4950
Wire Wire Line
	3200 6000 3300 6000
Text Label 2000 3850 0    50   ~ 0
-OC1
Text Label 2600 4950 0    50   ~ 0
-OC2
Text Label 3300 6000 0    50   ~ 0
-OC3
Text Label 4000 2500 0    50   ~ 0
sign
Wire Wire Line
	4650 3000 4500 3000
Text Label 4000 3000 0    50   ~ 0
+5V
Wire Wire Line
	4000 3000 4300 3000
$Comp
L Device:R_Small R9
U 1 1 5F462C82
P 4400 3000
F 0 "R9" V 4330 3000 50  0000 C CNN
F 1 "150" V 4400 3000 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" H 4400 3000 50  0001 C CNN
F 3 "~" H 4400 3000 50  0001 C CNN
	1    4400 3000
	0    1    1    0   
$EndComp
Wire Wire Line
	4650 3150 4000 3150
Text Label 4000 3150 0    50   ~ 0
OC1
Wire Wire Line
	4900 2500 4900 2700
Wire Wire Line
	3750 2500 3900 2500
Wire Wire Line
	6250 2850 6100 2850
Text Label 5600 2850 0    50   ~ 0
GND
Wire Wire Line
	5600 2850 5900 2850
$Comp
L Device:R_Small R15
U 1 1 5F51A3D6
P 6000 2850
F 0 "R15" V 5930 2850 50  0000 C CNN
F 1 "150" V 6000 2850 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" H 6000 2850 50  0001 C CNN
F 3 "~" H 6000 2850 50  0001 C CNN
	1    6000 2850
	0    1    1    0   
$EndComp
Wire Wire Line
	6250 3000 6100 3000
Text Label 5600 3000 0    50   ~ 0
GND
Wire Wire Line
	5600 3000 5900 3000
$Comp
L Device:R_Small R16
U 1 1 5F51A3E3
P 6000 3000
F 0 "R16" V 5930 3000 50  0000 C CNN
F 1 "150" V 6000 3000 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" H 6000 3000 50  0001 C CNN
F 3 "~" H 6000 3000 50  0001 C CNN
	1    6000 3000
	0    1    1    0   
$EndComp
Wire Wire Line
	6250 3150 5600 3150
Text Label 5600 3150 0    50   ~ 0
OC2
Wire Wire Line
	5100 3000 5300 3000
Text Label 5150 3000 0    50   ~ 0
FROMlo
Wire Wire Line
	6700 3000 6900 3000
Text Label 6750 3000 0    50   ~ 0
FROMmi
Wire Wire Line
	4900 2500 6500 2500
Wire Wire Line
	6500 2500 6500 2700
Connection ~ 4900 2500
Wire Wire Line
	7900 2850 7750 2850
Text Label 7250 2850 0    50   ~ 0
GND
Wire Wire Line
	7250 2850 7550 2850
$Comp
L Device:R_Small R21
U 1 1 5F657686
P 7650 2850
F 0 "R21" V 7580 2850 50  0000 C CNN
F 1 "150" V 7650 2850 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" H 7650 2850 50  0001 C CNN
F 3 "~" H 7650 2850 50  0001 C CNN
	1    7650 2850
	0    1    1    0   
$EndComp
Wire Wire Line
	7900 3000 7750 3000
Text Label 7250 3000 0    50   ~ 0
GND
Wire Wire Line
	7250 3000 7550 3000
$Comp
L Device:R_Small R22
U 1 1 5F657693
P 7650 3000
F 0 "R22" V 7580 3000 50  0000 C CNN
F 1 "150" V 7650 3000 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" H 7650 3000 50  0001 C CNN
F 3 "~" H 7650 3000 50  0001 C CNN
	1    7650 3000
	0    1    1    0   
$EndComp
Wire Wire Line
	7900 3150 7250 3150
Text Label 7250 3150 0    50   ~ 0
OC3
Wire Wire Line
	8350 3000 8550 3000
Text Label 8400 3000 0    50   ~ 0
FROMhi
Wire Wire Line
	6500 2500 8150 2500
Wire Wire Line
	8150 2500 8150 2700
Connection ~ 6500 2500
Wire Wire Line
	4650 4150 4500 4150
Text Label 4000 4150 0    50   ~ 0
+5V
Wire Wire Line
	4000 4150 4300 4150
$Comp
L Device:R_Small R13
U 1 1 5F74D821
P 4400 4150
F 0 "R13" V 4330 4150 50  0000 C CNN
F 1 "150" V 4400 4150 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" H 4400 4150 50  0001 C CNN
F 3 "~" H 4400 4150 50  0001 C CNN
	1    4400 4150
	0    1    1    0   
$EndComp
Wire Wire Line
	4650 4300 4500 4300
Text Label 4000 4300 0    50   ~ 0
+5V
Wire Wire Line
	4000 4300 4300 4300
$Comp
L Device:R_Small R14
U 1 1 5F74D82E
P 4400 4300
F 0 "R14" V 4330 4300 50  0000 C CNN
F 1 "150" V 4400 4300 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" H 4400 4300 50  0001 C CNN
F 3 "~" H 4400 4300 50  0001 C CNN
	1    4400 4300
	0    1    1    0   
$EndComp
Wire Wire Line
	4650 4000 4000 4000
Text Label 4000 4000 0    50   ~ 0
-OC1
Wire Wire Line
	6250 4150 6100 4150
Text Label 5600 4150 0    50   ~ 0
GND
Wire Wire Line
	5600 4150 5900 4150
$Comp
L Device:R_Small R17
U 1 1 5F7B8D60
P 6000 4150
F 0 "R17" V 5930 4150 50  0000 C CNN
F 1 "150" V 6000 4150 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" H 6000 4150 50  0001 C CNN
F 3 "~" H 6000 4150 50  0001 C CNN
	1    6000 4150
	0    1    1    0   
$EndComp
Wire Wire Line
	6250 4300 6100 4300
Text Label 5600 4300 0    50   ~ 0
GND
Wire Wire Line
	5600 4300 5900 4300
$Comp
L Device:R_Small R20
U 1 1 5F7B8D6D
P 6000 4300
F 0 "R20" V 5930 4300 50  0000 C CNN
F 1 "150" V 6000 4300 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" H 6000 4300 50  0001 C CNN
F 3 "~" H 6000 4300 50  0001 C CNN
	1    6000 4300
	0    1    1    0   
$EndComp
Wire Wire Line
	6250 4000 5600 4000
Text Label 5600 4000 0    50   ~ 0
-OC2
Wire Wire Line
	7900 4150 7750 4150
Text Label 7250 4150 0    50   ~ 0
GND
Wire Wire Line
	7250 4150 7550 4150
$Comp
L Device:R_Small R23
U 1 1 5F7F027E
P 7650 4150
F 0 "R23" V 7580 4150 50  0000 C CNN
F 1 "150" V 7650 4150 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" H 7650 4150 50  0001 C CNN
F 3 "~" H 7650 4150 50  0001 C CNN
	1    7650 4150
	0    1    1    0   
$EndComp
Wire Wire Line
	7900 4300 7750 4300
Text Label 7250 4300 0    50   ~ 0
GND
Wire Wire Line
	7250 4300 7550 4300
$Comp
L Device:R_Small R24
U 1 1 5F7F028B
P 7650 4300
F 0 "R24" V 7580 4300 50  0000 C CNN
F 1 "150" V 7650 4300 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" H 7650 4300 50  0001 C CNN
F 3 "~" H 7650 4300 50  0001 C CNN
	1    7650 4300
	0    1    1    0   
$EndComp
Wire Wire Line
	7900 4000 7250 4000
Text Label 7250 4000 0    50   ~ 0
-OC3
Wire Wire Line
	5100 4150 5300 4150
Text Label 5150 4150 0    50   ~ 0
TOlo
Wire Wire Line
	6700 4150 6900 4150
Text Label 6750 4150 0    50   ~ 0
TOmi
Wire Wire Line
	8350 4150 8550 4150
Text Label 8400 4150 0    50   ~ 0
TOhi
Wire Wire Line
	3900 2500 3900 3700
Wire Wire Line
	3900 3700 4900 3700
Wire Wire Line
	4900 3700 4900 3850
Connection ~ 3900 2500
Wire Wire Line
	3900 2500 4900 2500
Wire Wire Line
	4900 3700 6500 3700
Wire Wire Line
	6500 3700 6500 3850
Connection ~ 4900 3700
Wire Wire Line
	6500 3700 8150 3700
Wire Wire Line
	8150 3700 8150 3850
Connection ~ 6500 3700
Wire Wire Line
	5050 6150 4700 6150
Wire Wire Line
	5050 5850 4950 5850
Wire Wire Line
	5050 6000 4950 6000
Wire Wire Line
	4950 6000 4950 5850
Connection ~ 4950 5850
Wire Wire Line
	4950 5850 4700 5850
Text Label 4700 5850 0    50   ~ 0
FROMlo
Text Label 4700 6150 0    50   ~ 0
TOlo
Wire Wire Line
	5300 5400 5300 5550
Wire Wire Line
	5300 5550 6900 5550
Wire Wire Line
	6900 5550 6900 5700
Connection ~ 5300 5550
Wire Wire Line
	5300 5550 5300 5700
Wire Wire Line
	6900 5550 8550 5550
Wire Wire Line
	8550 5550 8550 5700
Connection ~ 6900 5550
Text Label 5300 5400 0    50   ~ 0
CYCLE
Wire Wire Line
	6650 6150 6300 6150
Wire Wire Line
	6650 5850 6550 5850
Wire Wire Line
	6650 6000 6550 6000
Wire Wire Line
	6550 6000 6550 5850
Connection ~ 6550 5850
Wire Wire Line
	6550 5850 6300 5850
Text Label 6300 5850 0    50   ~ 0
FROMmi
Text Label 6300 6150 0    50   ~ 0
TOmi
Wire Wire Line
	8300 6150 7950 6150
Wire Wire Line
	8300 5850 8200 5850
Wire Wire Line
	8300 6000 8200 6000
Wire Wire Line
	8200 6000 8200 5850
Connection ~ 8200 5850
Wire Wire Line
	8200 5850 7950 5850
Text Label 7950 5850 0    50   ~ 0
FROMhi
Text Label 7950 6150 0    50   ~ 0
TOhi
Wire Wire Line
	5500 6000 5750 6000
Wire Wire Line
	7100 6000 7400 6000
Wire Wire Line
	8750 6000 9100 6000
Text Label 5750 6000 0    50   ~ 0
Rlo
Text Label 7400 6000 0    50   ~ 0
Rmi
Text Label 9100 6000 0    50   ~ 0
Rhi
Wire Wire Line
	9450 800  10000 800 
Wire Wire Line
	9450 1100 10100 1100
Wire Wire Line
	9450 1350 10150 1350
Wire Wire Line
	9450 1650 10050 1650
Text Label 1550 6750 0    50   ~ 0
OC1
Wire Wire Line
	5650 1100 7900 1100
Wire Wire Line
	5750 1050 7950 1050
Wire Wire Line
	5850 1000 7550 1000
$EndSCHEMATC
