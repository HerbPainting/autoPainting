import operator

class HRRTTree(object):
    
    def __init__(self, planning_env, start_config,start_cost, start_total_cost):
        
        self.planning_env = planning_env
        self.vertices = []
        self.vertices.append(start_config)
        self.edges = dict()
        self.costs = []
        self.costs.append(start_cost)
        self.total_costs = []
        self.total_costs.append(start_total_cost)
        self.prob_floor = 0.3

    def GetRootId(self):
        return 0

    def GetNearestVertex(self, config):
        
        dists = []
        for v in self.vertices:
            dists.append(self.planning_env.ComputeDistance(config, v))

        vid, vdist = min(enumerate(dists), key=operator.itemgetter(1))

        return vid, self.vertices[vid], self.costs[vid], self.total_costs[vid]
            

    def AddVertex(self, config,cost,total_cost):
        vid = len(self.vertices)
        self.vertices.append(config)
        self.costs.append(cost)
        self.total_costs.append(total_cost)
        
        return vid

    def AddEdge(self, sid, eid):
        self.edges[eid] = sid

    def getMaxCost(self):
        
        return max(self.total_costs)

