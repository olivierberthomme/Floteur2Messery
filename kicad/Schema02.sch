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
Wire Wire Line
	3650 2100 3650 2400
Wire Wire Line
	3650 2400 4000 2400
Wire Wire Line
	3050 2100 3650 2100
Wire Wire Line
	3050 2250 3050 2100
Wire Wire Line
	3000 2250 3050 2250
Wire Wire Line
	5500 1450 5500 3000
Wire Wire Line
	2700 1450 5500 1450
Wire Wire Line
	4600 2200 4600 2250
Wire Wire Line
	4000 3100 4600 3100
Connection ~ 4000 3100
Wire Wire Line
	4000 3000 4000 3100
Wire Wire Line
	4600 3100 4850 3100
Connection ~ 4600 3100
Wire Wire Line
	2600 3100 4000 3100
Wire Wire Line
	3000 1950 5300 1950
Connection ~ 5300 3100
Wire Wire Line
	5300 3100 5500 3100
Wire Wire Line
	5300 1950 5300 3100
Wire Wire Line
	2600 3100 2600 3050
$Comp
L Device:R R2
U 1 1 5F5ABE3D
P 5000 3100
F 0 "R2" V 5207 3100 50  0000 C CNN
F 1 "R" V 5116 3100 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0309_L9.0mm_D3.2mm_P12.70mm_Horizontal" V 4930 3100 50  0001 C CNN
F 3 "~" H 5000 3100 50  0001 C CNN
	1    5000 3100
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5150 3100 5300 3100
$Comp
L Device:R R1
U 1 1 5F5AC5A8
P 4000 2850
F 0 "R1" H 4070 2896 50  0000 L CNN
F 1 "10 Ohm" H 4070 2805 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0309_L9.0mm_D3.2mm_P12.70mm_Horizontal" V 3930 2850 50  0001 C CNN
F 3 "~" H 4000 2850 50  0001 C CNN
	1    4000 2850
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x02 J1
U 1 1 5F5AB257
P 5700 3000
F 0 "J1" H 5780 2992 50  0000 L CNN
F 1 "Conn_01x02" H 5780 2901 50  0000 L CNN
F 2 "Connector_Wire:SolderWire-0.25sqmm_1x02_P4.5mm_D0.65mm_OD2mm" H 5700 3000 50  0001 C CNN
F 3 "~" H 5700 3000 50  0001 C CNN
	1    5700 3000
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D1
U 1 1 5F5A9FC4
P 4000 2550
F 0 "D1" V 4039 2432 50  0000 R CNN
F 1 "LED Bleue" V 3948 2432 50  0000 R CNN
F 2 "Diode_THT:D_DO-201AD_P5.08mm_Vertical_KathodeUp" H 4000 2550 50  0001 C CNN
F 3 "~" H 4000 2550 50  0001 C CNN
	1    4000 2550
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED D2
U 1 1 5F5A9938
P 4600 2400
F 0 "D2" V 4639 2282 50  0000 R CNN
F 1 "LED Rouge" V 4548 2282 50  0000 R CNN
F 2 "Diode_THT:D_DO-201AD_P5.08mm_Vertical_KathodeUp" H 4600 2400 50  0001 C CNN
F 3 "~" H 4600 2400 50  0001 C CNN
	1    4600 2400
	0    -1   -1   0   
$EndComp
$Comp
L MCU_Module:WeMos_D1_mini U1
U 1 1 5F5A81C5
P 2600 2250
F 0 "U1" H 2600 1361 50  0000 C CNN
F 1 "WeMos_D1_mini" H 2600 1270 50  0000 C CNN
F 2 "Module:WEMOS_D1_mini_light" H 2600 1100 50  0001 C CNN
F 3 "https://wiki.wemos.cc/products:d1:d1_mini#documentation" H 750 1100 50  0001 C CNN
	1    2600 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	3000 2550 3650 2550
Wire Wire Line
	3650 2550 3650 2400
Connection ~ 3650 2400
Wire Wire Line
	3000 2450 3250 2450
Wire Wire Line
	3250 2450 3250 2200
Wire Wire Line
	3250 2200 4600 2200
$Comp
L Device:R R?
U 1 1 5F708C84
P 4600 2850
F 0 "R?" H 4670 2896 50  0000 L CNN
F 1 "10 Ohm" H 4670 2805 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0309_L9.0mm_D3.2mm_P12.70mm_Horizontal" V 4530 2850 50  0001 C CNN
F 3 "~" H 4600 2850 50  0001 C CNN
	1    4600 2850
	1    0    0    -1  
$EndComp
Wire Wire Line
	4600 3000 4600 3100
Wire Wire Line
	4600 2550 4600 2700
$EndSCHEMATC
