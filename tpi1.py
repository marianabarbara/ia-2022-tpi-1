from tree_search import *
from cidades import *

class MyNode(SearchNode):
    def __init__(self,state,parent, cost, heuristic, depth = None):
        super().__init__(state,parent)
        self.heuristic = heuristic
        self.cost = cost
        self.depth = depth
        self.children = []
        self.eval = 0
        
class MyTree(SearchTree):

    def __init__(self,problem, strategy='breadth',seed=0): 
        super().__init__(problem,strategy,seed)
        self.problem = problem
        self.terminals = 0
        self.non_terminals = 0
        root = MyNode(problem.initial, None, 0, self.problem.domain.heuristic(problem.initial, problem.goal), 0)
        self.all_nodes = [root]
        self.solution_tree = None

    def astar_add_to_open(self,lnewnodes):
        #IMPLEMENT HERE
        self.open_nodes=sorted(self.open_nodes + lnewnodes, key =lambda pos:(self.all_nodes[pos]).heuristic+(self.all_nodes[pos]).cost)

    def propagate_eval_upwards(self,node):
        #IMPLEMENT HERE
        if node.children == []:
            return self.propagate_eval_upwards(self.all_nodes[node.parent]) #Nó pai => nao tem filhos
        
        children_eval = []
        
        for v in node.children:
            children_eval.append(self.all_nodes[v].eval)
           
        children_eval.sort(key = lambda eval: eval)
        node.eval = children_eval.pop(0)
            

        if node.parent == None:
            return None

        return self.propagate_eval_upwards(self.all_nodes[node.parent])

    def search2(self,atmostonce=False):
        #IMPLEMENT HERE
        while self.open_nodes != []:
            nodeID = self.open_nodes.pop(0)
            node = self.all_nodes[nodeID]
            
            if self.problem.goal_test(node.state):
                self.solution = node
                self.terminals = len(self.open_nodes)+1
                self.cost = self.solution.cost
                return self.get_path(node)
            
            lnewnodes = []
            self.non_terminals += 1
            
            for a in self.problem.domain.actions(node.state):
                newstate = self.problem.domain.result(node.state,a)
                
                if newstate not in self.get_path(node):
                    newcost =  self.problem.domain.cost(node.state, a) + node.cost
                    newnode = MyNode(newstate, nodeID, newcost , self.problem.domain.heuristic(newstate, self.problem.goal))
                    newnode.cost = node.cost + self.problem.domain.cost(node.state, a)
                    newnode.eval = newnode.cost + newnode.heuristic 
                    node.children.append(len(self.all_nodes))
                    self.all_nodes.append(newnode)
                    self.propagate_eval_upwards(newnode)
                    lnewnodes.append(len(self.all_nodes) - 1)
                    
            self.add_to_open(lnewnodes)
            
        return None

    def repeated_random_depth(self,numattempts=3,atmostonce=False):
        #IMPLEMENT HERE
        minimumCost = float('inf')
       
        for v in range(0, numattempts):
            newtree = MyTree(self.problem, 'rand_depth', v)
            newtree.search2()
           
            if newtree.solution.cost < minimumCost:
                self.solution_tree = newtree
                minimumCost = newtree.solution.cost
            
        return self.solution_tree.get_path(self.solution_tree.solution)

    def make_shortcuts(self):
        #IMPLEMENT HERE
        shortcut = self.get_path(self.solution)
        
        for v in range(0, len(shortcut)):
            
            if shortcut[v] != '0':
                connection = self.problem.domain.actions(shortcut[v])
                
                for w in connection:
                    size = len(shortcut) - 1
                   
                    while size > 1:
                       
                        if self.problem.domain.result(shortcut[v], w) == shortcut[size] and size - v > 1 and shortcut[size] != '0' :
                            self.used_shortcuts += [(shortcut[v], shortcut[size])]
                            shortcut[v + 1: size] = '0' 

                        size -= 1

            v += 1
            
            return  list(filter(lambda x: x != '0', shortcut))




class MyCities(Cidades):

    def maximum_tree_size(self,depth):   # assuming there is no loop prevention
        #IMPLEMENT HERE
        avg_branching = 0
        depth_final = 0
        
        for cidade in self.coordinates:   
            
            for connection in self.connections:
               
                if cidade in connection:
                    avg_branching += 1

        for x in range (0, depth + 1):
            depth_final += (avg_branching / len(self.coordinates)) ** x

        return depth_final


