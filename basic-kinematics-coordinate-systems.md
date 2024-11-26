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

<svg viewBox="0 0 400 300" xmlns="http://www.w3.org/2000/svg">
    <!-- Base -->
    <circle cx="100" cy="200" r="10" fill="#666"/>
    
    <!-- First link -->
    <line x1="100" y1="200" x2="200" y2="150" stroke="#444" stroke-width="8"/>
    <circle cx="200" cy="150" r="6" fill="#888"/>
    
    <!-- Second link -->
    <line x1="200" y1="150" x2="280" y2="120" stroke="#444" stroke-width="6"/>
    <circle cx="280" cy="120" r="4" fill="#888"/>
    
    <!-- Angles -->
    <path d="M 110,200 A 20,20 0 0 0 115,180" fill="none" stroke="#F00" stroke-width="2"/>
    <text x="120" y="190" fill="#F00">θ₁</text>
    
    <path d="M 210,150 A 20,20 0 0 0 220,135" fill="none" stroke="#F00" stroke-width="2"/>
    <text x="220" y="145" fill="#F00">θ₂</text>
    
    <!-- Coordinates -->
    <text x="285" y="115">(x, y)</text>
    
    <!-- Link lengths -->
    <text x="140" y="160">L₁</text>
    <text x="230" y="120">L₂</text>
    
    <!-- Coordinate system -->
    <line x1="100" y1="200" x2="150" y2="200" stroke="#000" stroke-width="1"/>
    <line x1="100" y1="200" x2="100" y2="150" stroke="#000" stroke-width="1"/>
    <text x="155" y="205">x</text>
    <text x="95" y="145">y</text>
</svg>

Input: Joint angles (θ₁, θ₂)
Output: End-effector position (x, y)

x = L₁cos(θ₁) + L₂cos(θ₁ + θ₂)
y = L₁sin(θ₁) + L₂sin(θ₁ + θ₂)

