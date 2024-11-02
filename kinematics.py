import math

class Kinematics:
    def __init__(self, linkLength1, linkLength2, linkLength3):
        self.linkLength1 = linkLength1
        self.linkLength2 = linkLength2
        self.linkLength3 = linkLength3

    def forward(self, jointAngle1, jointAngle2, jointAngle3):
        angleA = math.radians(jointAngle1)
        angleB = math.radians(jointAngle2)
        angleC = math.radians(jointAngle3)

        baseX, baseY = 0.0, 0.0

        joint1X = self.linkLength1 * math.cos(angleA)
        joint1Y = self.linkLength1 * math.sin(angleA)

        joint2X = joint1X + self.linkLength2 * math.cos(angleA + angleB)
        joint2Y = joint1Y + self.linkLength2 * math.sin(angleA + angleB)
        
        endEffectorX = joint2X + self.linkLength3 * math.cos(angleA + angleB + angleC)
        endEffectorY = joint2Y + self.linkLength3 * math.sin(angleA + angleB + angleC)
        
        return baseX, baseY, joint1X, joint1Y, joint2X, joint2Y, endEffectorX, endEffectorY    

    def inverse(self, targetX, targetY, targetT):
        angle = math.radians(targetT)
        x2 = targetX - self.linkLength3 * math.cos(angle)
        y2 = targetY - self.linkLength3 * math.sin(angle)

        # Calculate theta2 using cosine law
        c2 = ((x2**2) + (y2**2) - self.linkLength1**2 - self.linkLength2**2) / (2 * self.linkLength1 * self.linkLength2)
        if c2 < -1 or c2 > 1:
            raise ValueError("Position is unreachable with the given arm lengths.")

        theta2 = math.acos(c2)
        s2 = math.sqrt(1 - c2**2)

        # Calculate theta1
        k1 = self.linkLength1 + self.linkLength2 * c2
        k2 = self.linkLength2 * s2

        theta1 = math.atan2(y2, x2) - math.atan2(k2, k1)
        
        # Calculate theta3
        theta3 = angle - theta1 - theta2

        # Convert angles to degrees for easier interpretation
        theta1_deg = math.degrees(theta1)
        theta2_deg = math.degrees(theta2)
        theta3_deg = math.degrees(theta3)

        return theta1_deg, theta2_deg, theta3_deg