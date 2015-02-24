#!/usr/bin/env python3

import math

def binarySolveStep3(dist, v2, a, maxT, J):
    step = maxT / 2
    t = step
    for i in range(10):
        step /= 2
        d_trial = t*v2 + a*t*t/2 - J*t*t*t/6
        if d_trial < dist:
            t += step
        else:
            t -= step
    return t
        
class Profile:
    def __init__(self, totalDist, V=1.0, A=1.0, J=1.0):
        self.A = A
        self.J = J
        self.totalDist = totalDist
        dist = self.totalDist / 2

        riseTime = (V/A-A/J)
        self.maxV = V
        if riseTime >= 0:
            self.fullRampDist = V*(V/A+A/J)/2
            self.threestage = True
        else:
            self.fullRampDist = math.sqrt(V / J) * V
            self.threestage = False

        if dist < self.fullRampDist:
            maxV1 = A * ( - A / J + math.sqrt( (A*A)/(J*J) + 8 * dist / A)) / 2
            maxV2 = math.pow(dist**2*J, 1/3)
            self.fullRampDist = dist
            self.threestage = maxV1 > maxV2
            self.maxV = max(maxV1, maxV2)
            
        if self.threestage:
            self.v1 = 1/2*A*A/J
            self.v2 = self.maxV - self.v1
            self.d1 = 1/6*A*A*A/J/J
            self.d2 = self.d1 + self.maxV*riseTime / 2
            self.iA = 1 / A
            self.AdJ = A / J
        else:
            self.tmid = math.pow(self.fullRampDist / J, 1/3)
            self.d1 = 1/6*J*self.tmid*self.tmid*self.tmid
            self.v1 = 1/2*J*self.tmid*self.tmid
            self.a1 = self.J * self.tmid

    def getParameters(self, distLeft):
        median = self.totalDist / 2
        if (distLeft > median):
            v,a,j = self._getLeadingParameters(self.totalDist - distLeft)
            return (v, -a, -j)
        else:
            v,a,j = self._getLeadingParameters(distLeft)
            return (v, a, j)
    
    def _getLeadingParameters(self, x):
        if x >= self.fullRampDist:
            return (self.maxV,0,0)
        
        if self.threestage:
            if x < self.d1:
                t = math.pow(6 * x / self.J, 1/3)
                return (1/2*self.J*t*t,self.J*t, self.J)
            elif x < self.d2:
                dz = x - self.d1
                tz = (-self.v1+math.sqrt(self.v1*self.v1+2*self.A*dz)) * self.iA
                return (self.v1 + self.A*tz, self.A, 0)
            else:#x < d3
                dz = x - self.d2
                tz = binarySolveStep3(dz, self.v2, self.A, self.AdJ, self.J)
                tq = self.AdJ - tz
                return (self.maxV - 1/2*self.J*tq*tq, self.J*tq, -self.J)
        else:
            if x < self.d1:
                t = math.pow(6 * x / self.J, 1/3)
                a = self.J*t
                v = 1/2*self.J*t*t
                return (v,a, self.J)
            else:
                dz = x - self.d1
                tz = binarySolveStep3(dz, self.v1, self.a1, self.tmid, self.J)
                v = self.v1+self.a1*tz-self.J*tz*tz/2
                a = self.a1 - self.J*tz
                return (v, a, -self.J)

def main():
    steps = 200
    totalDist = 2

    profile = Profile(totalDist, V=0.4, A=0.2, J=0.1)
    
    for i in range(steps):
        distLeft = totalDist * (1 - i / steps)
        v,a,j = profile.getParameters(distLeft)
        print("{: f} {: f} {: f} {: f}".format(distLeft, v,a,j))

if __name__ == "__main__":
    main()
