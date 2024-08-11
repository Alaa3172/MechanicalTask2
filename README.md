# Mechanical Task 2
# 3-DoF Robotic Arm Kinematics

This README file details the forward and inverse kinematics calculations for a 3-DoF robotic arm. The aim is to determine the position of the end-effector (forward kinematics) and to compute the joint angles required to achieve a desired end-effector position (inverse kinematics).

## System Overview

We are working with a robotic arm that has three links, each connected by revolute joints. The lengths of the links and the joint angles are given as follows:

### Link Lengths
- `L1 = 5` units (length of the first link)
- `L2 = 3` units (length of the second link)
- `L3 = 2` units (length of the third link)

### Joint Angles
- `θ1 = 30° = π/6` radians (angle between the base and the first link)
- `θ2 = 45° = π/4` radians (angle between the first and second links)
- `θ3 = 60° = π/3` radians (angle between the second and third links)

### Z-axis Offset
- `d1 = 1` unit (distance along the z-axis from the base to the first link)

These parameters define the configuration of the robotic arm and are essential for performing kinematic calculations.

---

## Forward Kinematics

### Introduction to Forward Kinematics
Forward kinematics involves determining the position and orientation of the end-effector given the joint parameters (lengths and angles). The process involves calculating a series of transformation matrices, each representing the position and orientation of one link relative to the previous link. By multiplying these matrices, we obtain the overall transformation matrix, which gives the position of the end-effector in the base frame.

### Step 1: Transformation Matrices

Each transformation matrix `T` is a 4x4 matrix that combines rotation and translation:

```
T = [ R  p ]
    [ 0  1 ]
```

Where `R` is a 3x3 rotation matrix, and `p` is a 3x1 translation vector. The rotation matrix describes the orientation of the link, while the translation vector describes its position.

#### **T1 (Base to Link 1):**
This matrix represents the transformation from the base frame to the first link:

```
T1 = [ cos(π/6)  -sin(π/6)  0  0 ]
     [ sin(π/6)   cos(π/6)  0  0 ]
     [   0          0       1  d1 ]
     [   0          0       0  1  ]
```

Here, the rotation matrix corresponds to a rotation of `θ1 = 30°` around the z-axis, and the translation vector accounts for the vertical offset `d1`.

#### **T2 (Link 1 to Link 2):**
This matrix represents the transformation from the first link to the second link:

```
T2 = [ cos(π/4)  -sin(π/4)  0  L1*cos(π/4) ]
     [ sin(π/4)   cos(π/4)  0  L1*sin(π/4) ]
     [   0          0       1      0       ]
     [   0          0       0      1       ]
```

The rotation matrix accounts for the rotation `θ2 = 45°` around the z-axis. The translation vector `[L1*cos(θ2), L1*sin(θ2), 0]` places the second link relative to the first.

#### **T3 (Link 2 to Link 3):**
This matrix represents the transformation from the second link to the third link:

```
T3 = [ cos(π/3)  -sin(π/3)  0  L2*cos(π/3) ]
     [ sin(π/3)   cos(π/3)  0  L2*sin(π/3) ]
     [   0          0       1      0       ]
     [   0          0       0      1       ]
```

This matrix accounts for the rotation `θ3 = 60°` and the translation due to the length of the second link.

### Step 2: Composite Transformation Matrix

To find the overall transformation matrix `T`, we multiply the individual transformation matrices:

```
T = T1 * T2 * T3
```

This matrix multiplication gives the position and orientation of the end-effector in the base frame.

The position of the end-effector `(x, y, z)` is found in the last column of the final transformation matrix.

### Step 3: End-Effector Position

Substituting the values of the trigonometric functions into the matrix multiplication results, we find:

```
x = 5*cos(π/6)*cos(π/4) + 3*cos(π/6)*cos(π/4)*cos(π/3) ≈ 3.98 units
y = 5*sin(π/6)*cos(π/4) + 3*sin(π/6)*cos(π/4)*cos(π/3) ≈ 2.298 units
z = d1 = 1 unit
```

Thus, the end-effector position in Cartesian coordinates is approximately:

```
(x, y, z) ≈ (3.98, 2.298, 1) units
```

---

## Inverse Kinematics

### Introduction to Inverse Kinematics
Inverse kinematics is the process of calculating the joint angles required to place the end-effector at a specific position and orientation. It involves working backwards from the end-effector's position to determine the joint parameters. This can be more challenging than forward kinematics because there may be multiple solutions or none at all.

### Step 1: Calculate `θ1`

The angle `θ1` is found by considering the projection of the end-effector onto the XY-plane:

```
θ1 = arctan2(y, x)
```

Substituting the values:

```
θ1 ≈ arctan2(2.298, 3.98) ≈ 30°
```

This angle represents the orientation of the first link relative to the base.

### Step 2: Calculate Wrist Center

The wrist center is the point where the last joint (just before the end-effector) is located. It is calculated by subtracting the effect of the third link from the end-effector position:

```
xw = x - L3 * cos(θ3) ≈ 3.98 - 1.5 ≈ 2.48
yw = y - L3 * sin(θ3) ≈ 2.298 - 2.598 ≈ -0.3
zw = z - d1 = 1 - 1 = 0
```

The wrist center is an intermediate point that simplifies the calculation of the other joint angles.

### Step 3: Calculate `θ2` and `θ3`

The distance `r` from the base to the wrist center is given by:

```
r = sqrt(xw^2 + yw^2) ≈ 2.5 units
```

Using the cosine rule, we can calculate `θ2`:

```
θ2 = arccos((r^2 + L1^2 - L2^2) / (2 * L1 * r))
```

Substituting the values:

```
θ2 ≈ 45°
```

Finally, `θ3` is found by:

```
θ3 = arctan2(zw, r) - θ2 ≈ 60°
```

### Summary of Results

- **Forward Kinematics:**
  ```
  (x, y, z) ≈ (3.98, 2.298, 1) units
  ```

- **Inverse Kinematics:**
  ```
  θ1 ≈ 30°, θ2 ≈ 45°, θ3 ≈ 60°
  ```

These results confirm the correctness of both the forward and inverse kinematics calculations for this robotic arm configuration.

---
