#!/usr/bin/env python3

import matplotlib.pyplot as plt

def solveRisingCubic(y_goal, x_min, x_max, a_3, a_2, a_1, a_0):
    step = (x_max - x_min) / 2
    x = (x_max + x_min) / 2
    target = y_goal - a_0
    for i in range(10):
        step /= 2
        y = ((a_3 * x + a_2)*x + a_1) * x
        if y < target:
            x += step
        else:
            x -= step
    return x

def solveQuadraticUpper(a_2, a_1, a_0):
    pass

class Pose():
    def __init__(self, X,V,A,J):
        self.X = X
        self.V = V
        self.A = A
        self.J = J
    def __repr__(self):
        return "({}, {}, {}, {})".format(self.X,self.V,self.A,self.J)

def distForS(v_i, v_e, A, J):
    return (v_e - v_i) / A + A / J * (v_e + v_i) / 2

class Profile():
    #profile starts at 0
    def __init__(self, xstart, xend, vstart, vend, maxV, maxA, maxJ):
        # velocities must stay within [0,maxV]
        if maxV <= 0 or maxA <= 0 or maxJ <= 0 or abs(vend) > maxV or abs(vstart) > maxV:
            self.valid = False
            return
        # final velocity must be in the correct direction
        if vend * (xend - xstart) < 0:
            self.valid = False
            return
        # todo: catch case where we cannot change from vstart to vend 
        # in the distance allowed under maxA/J/V
        self.valid = True
        
        # v values are taken with signs
        self.xstart = xstart
        self.xend = xend

        self.vstart = vstart
        self.vend = vend
        self.xdist = abs(xend - xstart)
        self.V = maxV
        self.A = maxA
        self.J = maxJ
        
        pass
    def isValid(self):
        return self.valid
    def solveHalfProblem(self, xdisp, dist, vstart):
        # follow path to maximize v at end with a=0
        # at xdisp=0, vstart
        # at xdisp=dist, A=0
        #assert xdisp >= 0 and xdisp <= dist, "xdisp out of range"
        assert vstart > 0, "vstart should be positive here"

        return xdisp*8, -xdisp*7, (dist-xdisp)*5
        
    def solveForwardProblem(self, xdisp, vstart, vend):
        #
        # Cases:
        # * Start velocity is greater than maxV (must slow down to V)
        # * We have more room than needed
        # * vstart < vend < maxV and the result is monotonic
        
        assert xdisp >= 0 and xdisp <= self.xdist, "xdisp out of range"
        
        #
        #
        #

        # rise_time is irrelevant
        # s_rise_dist is the distance needed assuming acceleration A is acheived
        # c_rise_dist is the distance needed assuming acceleration A is never acheived
        # f_rise_dist is the only possible choice of s_rise_dist and c_rise_dist
        
        left_s_rise_dist = (self.V - vstart) / self.A + self.A / self.J * (self.V + vstart) / 2
        left_c_rise_dist = math.sqrt((self.V - vstart) / self.J) * (self.V - vstart)
        right_s_rise_dist = (self.V - vend) / self.A + self.A / self.J      * (self.V + vend) / 2
        right_c_rise_dist = math.sqrt((self.V - vend) / self.J) * (self.V - vend)
        
        left_tri = True
        
        if (self.V - vstart) >= self.A**2/self.J:
            left_f_rise_dist = left_s_rise_dist
            left_tri = True
        else:
            left_f_rise_dist = left_c_rise_dist
            left_tri = False
        if (self.V - vend) >= self.A**2/self.J:
            right_f_rise_dist = right_s_rise_dist
            right_tri = True
        else:
            right_f_rise_dist = right_c_rise_dist
            right_tri = False
            
        print(left_opt_rise_dist, left_rise_time, left_trans_rise_dist, 
              right_opt_rise_dist, right_rise_time, right_trans_rise_dist)

        left_trans_rise_dist = (vstart + self.A**2/self.J)*self.A/self.J
        right_trans_rise_dist = (vend + self.A**2/self.J)*self.A/self.J
        
        if self.xdist >= left_f_rise_dist + right_f_rise_dist:
            # case: perfect w/ enought central steady space
            left_dist = left_f_rise_dist
            right_dist = right_f_rise_dist
        else:
            if left_trans_rise_dist + right_trans_rise_dist >= self.xdist:
                mode = "JJ"
                pass
            elif vstart > vend:
                if left_trans_rise_dist + distForS(v_end, vstart + (self.A**2*self.J), self.A, self.J) >= self.xdist:
                    mode = "AA"
                else:
                    mode = "JA"
            elif vend > vstart:
                if distForS(v_start, v_end + (self.A**2*self.J), self.A, self.J) + right_trans_rise_dist >= self.xdist:
                    mode = "AA"
                else:
                    mode = "AJ"
            else:
                mode = "AA"
            
            if mode == "AA":
                v_m = solveQuadraticUpper(
            elif mode == "AJ":
                pass
            elif mode == "JA":
                pass
            elif mode == "AA":
                pass
            else:
                assert False, "Not an option"
        
        
        left_dist = left_opt_rise_dist
        right_dist = right_opt_rise_dist
        
        # assert that: left_dist + right_dist <= xdist
        # goal is that: A at X >= left_dist && X <= xdist - right_dist == 0
        # The one exception case is the rare monotonic v1->v2 in a distance
              
        left_dist = right_dist = self.xdist / 3
        assert left_dist + right_dist <= self.xdist, "side distance division too large"
        
        if xdisp > left_dist and xdisp < self.xdist - right_dist:
            return self.V, 0, 0
        elif xdisp < left_dist:
            v,a,j = self.solveHalfProblem(xdisp, left_dist, vstart)
            return v,a,j
        elif xdisp > self.xdist - right_dist:
            v,a,j = self.solveHalfProblem(xdisp, left_dist, vstart)
            return -v, a, -j

        assert False, "case never happens"

    def byDist(self, xdist):
        if not self.isValid():
            return Pose(xdist, 0,0,0)
        if self.xstart > self.xend:
            if xdist > self.xstart:
                return Pose(xdist, self.vstart, 0, 0)
            if xdist < self.xend:
                return Pose(xdist, self.vend, 0, 0)
            v,a,j = self.solveForwardProblem(self.xstart - xdist, -self.vend, -self.vstart)
            return Pose(xdist, -v, a, -j)
        elif self.xstart < self.xend:
            if xdist < self.xstart:
                return Pose(xdist, self.vstart, 0, 0)
            if xdist > self.xend:
                return Pose(xdist, self.vend, 0, 0)
            v,a,j = self.solveForwardProblem(xdist - self.xstart, self.vstart, self.vend)
            return Pose(xdist, v,a,j)
        else:
            return Pose(xdist, 0,0,0)
    def byTime(self, time):
        return Pose(0, 0,0,0)
 
 
def create_image(xarr,yarrs, ylabels, name):
    plt.figure()
    for yarr, ylabel in zip(yarrs, ylabels):
        plt.plot(xarr, yarr, label=ylabel)
    plt.legend(loc="upper left")
    plt.xlabel("Position")
    
    xmx = max(xarr)
    xms = min(xarr)
    xr = xmx - xms
    plt.xlim(xms - xr * 0.05,xmx + xr * 0.05)
    
    ymx = max((max(yarr) for yarr in yarrs))
    yms = min((min(yarr) for yarr in yarrs))
    yr = ymx - yms
    plt.ylim(yms - yr * 0.05,ymx + yr * 0.05)
    
    plt.savefig(name)
    
def runTest(xvpairs, maxV, maxA, maxJ):
    overflow = 0.2
    steps = 600
    for xstart, xend, vstart, vend in xvpairs:
        prof = Profile(xstart, xend, vstart, vend, maxV, maxA, maxJ)
        xrstart = xstart - (xend - xstart) * overflow
        xrend = xend + (xend - xstart) * overflow
        xrng = xrend - xrstart
        marr = []
        for i in range(steps+1):
            x = i / steps * xrng + xrstart
            marr.append(prof.byDist(x))
        create_image( [m.X for m in marr], ([m.V for m in marr],[m.A for m in marr],[m.J for m in marr]), ("V","A","J"), "v{};a{};j{};va{};vb{};xa{};xb{};.png".format(maxV, maxA, maxJ, vstart, vend, xstart, xend))
    
if __name__ == "__main__":
    runTest( [(0.2,0.4,0.05,0.1)], 1.0, 1.0, 1.0)