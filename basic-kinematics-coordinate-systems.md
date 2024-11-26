# Basic Kinematics and coordinate systems

## Right-hand coordiante system

When pointing your right thumb along the positive z-axis, your fingers naturally curl in the direction from the positive x-axis toward the positive y-axis.

This convention is fundamental in:

Classical mechanics
Computer graphics
Robotics
Vector cross products
Many other fields of physics and engineering

## Forward kinematics
Use following 2-DOF planar example
![plot](./2D-planar-example.png)

Input: Joint angles (θ₁, θ₂)  
Output: End-effector position (x, y)  


```
x = L₁cos(θ₁) + L₂cos(θ₁ + θ₂)
y = L₁sin(θ₁) + L₂sin(θ₁ + θ₂)
```

