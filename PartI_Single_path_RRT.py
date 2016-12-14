
import getopt
import sys
import matplotlib.pyplot as plt
import matplotlib.path as mpt
import math
import numpy as np
import random
import path_finding as pf

'''
Program Structure based on the one proposed in the Paper
'''

#deltad = sys.argv[1]


class RRT:

    def __init__(self,num_K=10000,deltad=50):
        self.start = []
        self.goal = []
        self.obs = []
        self.tree_edges = []
        self.tree_vecs = []
        self.world = [600,600]
        self.K = num_K
        self.delta = deltad
        self.binary_cnt = 3


    def config(self,obs_file,sg_file):
        with open(sg_file,'r') as sg_f:
            for i in range(0,2):
                line = sg_f.readline()
                paras = line.split(' ')
                if i == 0:
                    self.start = [int(paras[0]),int(paras[1])]
                else:
                    self.goal = [int(paras[0]),int(paras[1])]
        with open(obs_file,'r') as obs_f:
            line = obs_f.readline()
            num_obs = int((line.split(' '))[0])
            print num_obs
            for i in range(0,num_obs):
                nline = obs_f.readline()
                num_vecs = int((nline.split(' '))[0])
                tmp_obs = []
                for j in range(0,num_vecs):
                    vline = obs_f.readline()
                    vecs = vline.split(' ')
                    tmp_obs.append([int(vecs[0]),int(vecs[1])])
                self.obs.append(tmp_obs)
        self.RRT_debug(0)
        return

    def start_draw(self):
        plt.ion()
        plt.scatter([self.start[0]],[self.start[1]],color='b')
        plt.scatter([self.goal[0]], [self.goal[1]], color='g')
        plt.draw()
        for i in range(0,len(self.obs)):
            x = []
            y = []
            for j in range(0,len(self.obs[i])):
                x.append(self.obs[i][j][0])
                y.append(self.obs[i][j][1])
            x.append(self.obs[i][0][0])
            y.append(self.obs[i][0][1])
            plt.plot(x, y, '.r-', linewidth=1)
            plt.pause(0.01)

    def end_draw(self):
        plt.pause(10)
        plt.close()

    def RRT_debug(self,flag=1):
        if flag < 1:
            print 'cur obs'
            for i in range(0,len(self.obs)):
                print self.obs[i]
        print 'cur vecs'
        for i in range(0,len(self.tree_vecs)):
            print self.tree_vecs[i]
        print 'cur edges'
        for i in range(0,len(self.tree_edges)):
            print self.tree_edges[i]


    def rand_state(self):
        rand_x = random.randint(0,600)
        rand_y = random.randint(0,600)
        return [rand_x,rand_y]

    def build_RRT(self):

        #Tree init
        self.tree_vecs.append(self.start)
        found = False
        for i in range(0,self.K):
            new_x = self.rand_state()
            if i % 20 == 0:
                new_x = self.goal
            #plt.plot(new_x[0], new_x[1], '.r-', linewidth=1)
            if self.extend_RRT(new_x):
                print 'Goal Reached'
                found = True
                break
        #plt.pause(10)
        if found:
            path = pf.dij(self.tree_edges,self.tree_vecs)
            x = []
            y = []
            for i in range(0,len(path)):
                x.append(path[len(path) - 1 - i][0])
                y.append(path[len(path) - 1 - i][1])
            plt.plot(x, y, '.g-', linewidth=2)
            plt.pause(0.01)
        return

    def nearest(self,point):
        min = sys.maxint
        loc = -1
        for i in range(0,len(self.tree_vecs)):
            dist = math.sqrt(pow(self.tree_vecs[i][0]-point[0],2)+pow(self.tree_vecs[i][1]-point[1],2))
            if dist < min:
                min = dist
                loc = i
        if loc == -1:
            print "Error Locating nearest Neighbor"
        else:
            return self.tree_vecs[loc]


    def getb_points(self,points,top,bot,cnt):
        if cnt != 0:
            mid = (top+bot)/2
            points.append(mid)
            self.getb_points(points,top,mid,cnt-1)
            self.getb_points(points, mid, bot, cnt - 1)
        else:
            return

    def get_binary_poits(self,top,bot):
        points = []
        self.getb_points(points,top,bot,self.binary_cnt)
        return points

    '''
    New state does the extension and checks for collsion
    '''
    def new_state(self,near,pos):
        orivec = [pos[0]-near[0],pos[1]-near[1]]
        dist = math.sqrt(pow(pos[0]-near[0],2)+pow(pos[1]-near[1],2))
        if dist == 0: #random point is a point on the tree, remove this random point
            return [False,[-1,-1]]
        ratio = self.delta / dist
        new = [ratio * orivec[0]+near[0],ratio * orivec[1]+near[1]]
        #print ratio
        for i in range(0,len(self.obs)):
            ob_path = mpt.Path(np.array(self.obs[i]))
            points = self.get_binary_poits(np.array(new), np.array(near))
            points.append(near)
            points.append(new)
            for i in range(0,len(points)):
                if ob_path.contains_point(points[i]):
                    return [False,[-1,-1]]
        if new[0] > 600 or new[1] > 600 or new[0] < 0 or new [1] < 0:
            return [False, [-1, -1]]

        return [True,new]

    def neargoal(self,pt):
        dist = math.sqrt(pow(self.goal[0] - pt[0], 2) + pow(self.goal[1] - pt[1], 2))
        if dist < 30:
            return True
        return False

    def extend_RRT(self,pos):
        nearest_vec = self.nearest(pos)
        #print pos
        res = self.new_state(nearest_vec,pos)
        if res[0] == True:
            self.tree_vecs.append(res[1])
            self.tree_edges.append([nearest_vec,res[1],self.delta])
            #print res[1]
            plt.plot([nearest_vec[0],res[1][0]],[nearest_vec[1],res[1][1]],'.y-',linewidth=1)
            plt.pause(0.001)
            if self.goal == res[1] or self.neargoal(res[1]):
                dist = math.sqrt(pow(self.goal[0] - res[1][0], 2) + pow(self.goal[1] - res[1][1], 2))
                self.tree_vecs.append(res[1])
                self.tree_vecs.append(self.goal)
                self.tree_edges.append([res[1],self.goal, dist])
                plt.plot([self.goal[0], res[1][0]], [self.goal[1], res[1][1]], '.y-', linewidth=1)
                plt.pause(0.001)
                return True

        return False
#print sys.argv[0]
#print len(sys.argv)
if len(sys.argv) < 2:
    myRRT = RRT()
else:
    print "Your step size "+sys.argv[1]
    myRRT = RRT(10000,int(sys.argv[1]))

myRRT.config('obstacle.txt','startgoal.txt')
myRRT.start_draw()
myRRT.build_RRT()
myRRT.end_draw()