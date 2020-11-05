# Help to design BLDC articulations
BLDC controllers are very new with respect the classical DC motor controllers, mainly taking into account position control.

## The facts

### A small motor (2204-2300KV) + Tinymovr controller - push test

the motor is set next to a weighing scale with a lever arm of 95mm. 
Input voltage : 15V.

#### Measures

| Position target (ticks)| Phase amps (A)| Input amps (A)| weight (g)|torque (N.m)|
| ---------------------- |:-------------:| -------------:|----------:|-----------:|
| 0                      | 0.918         | 0.060         |0          |0.000       |
| 1000                   | -0.165        | 0.060         |5          |0.005       |
| 2000                   | -3.640        | 0.120         |13         |0.012       |
| 3000                   | -6.112        | 0.230         |21         |0.020       |
| 4000                   | -8.710        | 0.420         |29         |0.028       |
| 5000                   | -10.040       | 0.570         |34         |0.032       |

#### Torque constant (controller included)
Kc = Torque/I = 0.032/0.570 = 0.056N.m/A


### A bigger motor (5005-280KV) + Tinymovr controller - push test

the motor is set next to a weighing scale with a lever arm of 98mm. 
Input voltage : 15V.

#### Measures

| Position target (ticks)| Phase amps (A)| Input amps (A)| weight (g)|torque (N.m)|
| ---------------------- |:-------------:| -------------:|----------:|-----------:|
| 0                      | 0.277         | 0.050         |0          |0.000       |
| 1000                   | -1.983        | 0.090         |50         |0.048       |
| 2000                   | -4.547        | 0.200         |112        |0.106       |
| 3000                   | -6.995        | 0.420         |175        |0.166       |
| 4000                   | -9.527        | 0.740         |236        |0.224       |
| 5000                   | -9.973        | 0.840         |247        |0.235       |
| 6000                   | -10.025       | 0.860         |249        |0.237       |

#### Torque constant (controller included)
Kc = Torque/I = 0.237/0.860 = 0.276N.m/A

### A small motor (2204-2300KV) + Tinymovr controller - speed test
the motor is without load

| Speed target (ticks)| Input amps (A)| Measured speed (ticks/s)|
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



