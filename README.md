# Help to design BLDC articulations
BLDC controllers are very new with respect the classical DC motor controllers, mainly taking into account position control. The objective of this repo is to gather studies to help design of robots (small and big) with BLDC.

## Motors

|                     |Motor 1 |Motor 2|
| ------------------- |:------:|:-----:|
|Picture|![](https://img.eachine.com//eachine/products/original/201605/1462515328_17.jpg)|![alt text](https://mad-motor.com/wp-content/uploads/2018/03/MAD5005-EEE_01.jpg "MAD")|
| Ref| [Eachine  2204-2300KV](https://www.eachine.com/Eachine-Racer-250-Drone-Spare-Part-BG2204-2300KV-Brushless-Motor-CW-or-CCW-p-343.html)|[MAD component 5005-280KV](https://mad-motor.com/product/mad-5005-eee/)|
|Resistance/phase(milliOhms)|170|230|
|Inductance/phase(microHenri)|10|76|
|Outer diameter(mm)|28|56|
|Max quadratic current Iq in coals|12 A|18 A|
|Purpose|Fast/silent animatronics (eye)|Powerful robotic articulation|

-----
## Tinymovr controller

[Documentation](https://tinymovr.readthedocs.io/en/latest/)
Here, the tinymovr controller board is powered by a DC power supply 15V - 10A.
The communication is done via a standard FTDI USB - UART device (for experiment purpose here... Later via CAN bus)
By default, Iq is limited to 10A and speed to 300000ticks/s
1 turn is 8192 ticks.

-----
## Theory about BLDC

### The motor current

Iq = (Ia/sqrt(3) + 2*Ib/sqrt(3))*cos(theta) - Ia*sin(theta)

### The torque constant

Torque (N.m) = Kc*Iq(A)

### The speed constant

U(V) = Kv*Omega(rad/s)

Knowing that Kc = 1/Kv

-----

### A small motor (2204-2300KV) + Tinymovr controller - push test

the motor is set next to a weighing scale with a lever arm of 95mm. 
Input voltage : 15V.

#### Measures

| Position target (ticks)| Motor current Iq (A)| Input current Iinput (A)| weight (g)|torque (N.m)|
| ---------------------- |:----------------:| -------------:|----------:|-----------:|
| 0                      | 0.918            | 0.060         |0          |0.000       |
| 1000                   | -0.165           | 0.060         |5          |0.005       |
| 2000                   | -3.640           | 0.120         |13         |0.012       |
| 3000                   | -6.112           | 0.230         |21         |0.020       |
| 4000                   | -8.710           | 0.420         |29         |0.028       |
| 5000                   | -10.040          | 0.570         |34         |0.032       |

Find below the input current of the Tinymovr controller function of the current delivered in coals.

![](./motor1-Iin-vs-Iphase.png)

Find below the torque generated by the motor function of the current delivered in coals

Kc = Torque/Iphase = 0.032/10.04 = 0.0032 N.m/A

Kv = 1/Kc = 310.8 rad/s/V

KV = 2970 rpm/V

![](./motor1-Torque-vs-Iphase.png)

And find below the torque produced function of input current

![](./motor1-Torque-vs-Iin.png)

### A bigger motor (5005-280KV) + Tinymovr controller - push test

![](./5005_setup.jpg)

the motor is set next to a weighing scale with a lever arm of 98mm. 
Input voltage : 15V.

#### Measures

| Position target (ticks)| Motor current Iq (A)| Input current Iinput (A)| weight (g)|torque (N.m)|
| ---------------------- |:-------------:| -------------:|----------:|-----------:|
| 0                      | 0.277         | 0.050         |0          |0.000       |
| 1000                   | -1.983        | 0.090         |50         |0.048       |
| 2000                   | -4.547        | 0.200         |112        |0.106       |
| 3000                   | -6.995        | 0.420         |175        |0.166       |
| 4000                   | -9.527        | 0.740         |236        |0.224       |
| 5000                   | -9.973        | 0.840         |247        |0.235       |
| 6000                   | -10.025       | 0.860         |249        |0.237       |

Find below the input current of the Tinymovr controller function of the current delivered in coals.

![](./motor2-Iin-vs-Iphase.png)

Find below the torque generated by the motor function of the current delivered in coals

Kc = Torque/Iphase = 0.032/10.04 = 0.0234 N.m/A

Kv = 1/Kc = 42.7 rad/s/V

KV = 407 rpm/V

![](./motor2-Torque-vs-Iphase.png)

And find below the torque produced function of input current

![](./motor2-Torque-vs-Iin.png)

----

### A small motor (2204-2300KV) + Tinymovr controller - speed test

the motor is without load

![](./2204_setup.jpg)

| Speed target (ticks/s)| Input amps (A)| Measured speed (ticks/s)|
| ------------------- |:-------------:| -----------------------:|
| 0                   | 0.060         | 168                     |
| 7000                | 0.060         | 506                     |
| 8000                | 0.060         | 2373                    |
| 10000               | 0.060         | 9880                    |
| 20000               | 0.060         | 17605                   |
| 40000               | 0.060         | 38343                   |
| 80000               | 0.060         | 79606                   |
| 160000              | 0.070         | 147904                  |
| 320000              | 0.080         | 278386                  |
| 640000              | 0.090         | 281503                  |

See that until 10000 ticks/s, the measured speed is far from the commanded one. The rotor slips and is too slow.
On the contrary, the high speed does not go above 300000 ticks/s. This is nominal since the speed is limited (can be tuned in Tinymovr)

### A bigger motor (2204-2300KV) + Tinymovr controller - speed test

the motor is without load

| Speed target (ticks/s)| Input amps (A)| Measured speed (ticks/s)|
| ------------------- |:-------------:| -------------------------:|
| 0                   | 0.060         | -0                        |
| 5000                | 0.060         | -292                      |
| 6000                | 0.060         | 4173                      |
| 7000                | 0.060         | 4915                      |
| 8000                | 0.060         | 5408                      |
| 9000                | 0.060         | 7836                      |
| 10000               | 0.060         | 8835                      |
| 20000               | 0.090         | 75905                     |
| 160000              | 0.140         | 166808                    |
| 320000              | 0.470         | 270000                    |

See that the motor can reach slower speeds even if it slips.