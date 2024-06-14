import numpy as np

class LegGait:
    
    def __init__(self, L, clearance_height, P0, vel, delta):
        
        self.delta = delta
        self.vel = vel
        self.L = L
        self.P0 = P0

        self.Tst = 2*L/vel
        self.Tsw = self.Tst*1.4166
        self.Ttot = self.Tst + self.Tsw

        self.Px = np.array([
            -L,  # Ctrl Point 0, half of stride len
            -L * 1.4,  # Ctrl Point 1 diff btwn 1 and 0 = x Lift vel
            -L * 1.5,  # Ctrl Pts 2, 3, 4 are overlapped for
            -L * 1.5,  # Direction change after
            -L * 1.5,  # Follow Through
            0.0,  # Change acceleration during Protraction
            0.0,  # So we include three
            0.0,  # Overlapped Ctrl Pts: 5, 6, 7
            L * 1.5,  # Changing direction for swing-leg retraction
            L * 1.5,  # requires double overlapped Ctrl Pts: 8, 9
            L * 1.4,  # Swing Leg Retraction Velocity = Ctrl 11 - 10
            L])
        
        self.NumControlPoints = 11

        self.Pz = np.array([
            0.0,  # Double Overlapped Ctrl Pts for zero Lift
            0.0,  # Veloicty wrt hip (Pts 0 and 1)
            clearance_height * 0.9,  # Triple overlapped control for change in
            clearance_height * 0.9,  # Force direction during transition from
            clearance_height * 0.9,  # follow-through to protraction (2, 3, 4)
            clearance_height * 0.9,  # Double Overlapped Ctrl Pts for Traj
            clearance_height * 0.9,  # Dirctn Change during Protraction (5, 6)
            clearance_height * 1.1,  # Maximum Clearance at mid Traj, Pt 7
            clearance_height * 1.1,  # Smooth Transition from Protraction
            clearance_height * 1.1,  # To Retraction, Two Ctrl Pts (8, 9)
            0.0,  # Double Overlap Ctrl Pts for 0 Touchdown
            0.0,  # Veloicty wrt hip (Pts 10 and 11)
            ]) - P0

    def BernSteinPoly(self, t, k, point):
        """Calculate the point on the Berinstein Polynomial
           based on phase (0->1), point number (0-11),
           and the value of the control point itself
           :param t: phase
           :param k: point number
           :param point: point value
           :return: Value through Bezier Curve
        """
        return point * self.Binomial(k) * np.power(t, k) * np.power(
            1 - t, self.NumControlPoints - k)

    def Binomial(self, k):
        """Solves the binomial theorem given a Bezier point number
           relative to the total number of Bezier points.
           :param k: Bezier point number
           :returns: Binomial solution
        """
        return np.math.factorial(self.NumControlPoints) / (
            np.math.factorial(k) *
            np.math.factorial(self.NumControlPoints - k))
    
    def move(self, t):
        timed = t % self.Ttot

        pos_x = 0
        pos_z = 0

        if timed <= self.Tst:
            pos_x = -self.vel*timed + self.L
            pos_z = -self.delta * np.cos(np.pi*(0.5 - self.vel*timed/(self.L*2))) - self.P0
        else:
            phase = (timed - self.Tst)/self.Tsw
            for i in range(self.NumControlPoints+1):
                pos_x += self.BernSteinPoly(phase, i, self.Px[i])
                pos_z += self.BernSteinPoly(phase, i, self.Pz[i])
        
        return pos_x, pos_z