import numpy as np

mass = 0.7 # kg
g = 9.81 # m/s/s
#I = np.array([(0.00372, 0, 2.55e-6),
#              (0, 0.00276, 0),
#              (2.55e-6, 0, 0.0044)]);
#I = np.array([(0.00372, 0, 0),
#              (0, 0.00372, 0),
#              (0, 0, 0.0044)]);
I = np.array([[0.00178117, 0.,         0.        ],
              [0.,         0.00257335, 0.        ],
              [0.,         0.,         0.00257335]]);

invI = np.linalg.inv(I)
arm_length = 0.125 # meter
height = 0.05
#minF = 0.0
#maxF = 2.0 * mass * g
L = arm_length
H = height
km = 1.5e-9
kf = 6.11e-8
r = km / kf
MotorF = 10

#  [ F  ]         [ F1 ] 
#  | M1 |  = A *  | F2 | 
#  | M2 |         | F3 |
#  [ M3 ]         [ F4 ]
# M1 Roll, M2 Pitch, M3 Yaw
A = np.array([[ 1,  1,   1,   1],
              [ L,  L,  -L,  -L],
              [ -L, L,  -L,  L],
              [ -r, r,  r,   -r]])
invA = np.linalg.inv(A)



body_frame = 2*np.array([(L, L, 0, 1),
                       (-L, L, 0, 1),
                       (-L, -L, 0, 1),
                       (L, -L, 0, 1),
                       (0, 0, 0, 1),
                       (0, 0, H, 1)])

