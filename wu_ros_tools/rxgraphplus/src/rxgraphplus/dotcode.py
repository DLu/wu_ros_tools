import roslib.names
import rosgraph.impl.graph
import collections

ORIENTATIONS = ['LR', 'TB', 'RL', 'BT']

INIT_DOTCODE = """
digraph G { initializing [label="initializing..."]; }
"""

# node/node connectivity
NODE_NODE_GRAPH = "node_node"
# node/topic connections where an actual network connection exists
NODE_TOPIC_GRAPH = "node_topic"
# all node/topic connections, even if no actual network connection
NODE_TOPIC_ALL_GRAPH = "node_topic_all" 

import urllib
def safe_dotcode_name(name):
    """
    encode the name for dotcode symbol-safe syntax    
    """
    # not the best solution, but unsafe names shouldn't be coming through much
    ret = urllib.quote(name)
    ret = ret.replace('/', '_')
    ret = ret.replace('%', '_')
    ret = ret.replace('-', '_')
    return ret

#TODO: Make param
QUIET_NAMES = ['/diag_agg', '/runtime_logger', '/pr2_dashboard', '/rviz', '/rosout', '/cpu_monitor', '/monitor','/hd_monitor', '/rxloggerlevel', '/clock']

class Graph:
    def __init__(self, orientation, quiet):
        self.nodes = {}
        self.edges = []
        self.orientation = orientation
        self.quiet = quiet
        self.parents = {}
        self.children = collections.defaultdict(list)
        self.group_topics = {}

    def add_node(self, o):
        self.nodes[ o.name ] = o

    def set_hierarchy(self, config, parent=None):
        for group, value in config.iteritems():
            if parent is not None:
                self.parents[group] = parent
            self.children[parent].append(group)
            if type(value)==type([]):
                for v in value:
                    if v in self.nodes:
                        self.parents[v] = group
                        self.children[group].append(v)
                    else:
                        self.group_topics[group] = v
            else:
                if 'topics' in value:
                    for v in value['topics']:
                        self.group_topics[group] = v
                    del value['topics']
                    
                self.set_hierarchy(value, group)

    def dot_code(self):
        if len(self.nodes)==0:
            nodes_str = '  empty;' 
        else:
            nodes_str = self.node_dot_code() 

        edges_str =     '\n'.join([e.dot_code(self) for e in self.edges          if self.is_valid_edge(e) ])

        s = "digraph G {\n  rankdir=%s;\ncompound=true; \n"%self.orientation
        s += "%(nodes_str)s\n%(edges_str)s}\n"%vars()
        return s

    def is_valid_node(self, node):
        if self.quiet:
            return node.name not in QUIET_NAMES
        return True

    def is_valid_edge(self, edge):
        start = self.nodes[edge.start]
        end =   self.nodes[edge.end]
        if not (self.is_valid_node(start) and self.is_valid_node(end)):
            return False
        if self.same_parents(edge):
            return False
        return True

    def same_parents(self, edge):
        return edge.start in self.parents and edge.end in self.parents and self.parents[edge.start]==self.parents[edge.end]

    def node_dot_code(self, parent=None, depth=1):
        s = ''
        std = ' '*depth
        tab = ' '*(depth+1)

        if parent is not None:
            color = 'grey%d'%(100-(depth-1)*10)
            s += std + 'subgraph %s {\n' % safe_dotcode_name('cluster' + parent)
            s += tab + 'label = "%s";\n'%parent
            s += tab + 'style=filled;\n'
            s += tab + 'fillcolor=%s;\n'%color

        for child in self.children[parent]:
            if child in self.nodes:
                if self.is_valid_node(self.nodes[child]):
                    s += self.nodes[child].dot_code()            
            else:
                s += self.node_dot_code(child, depth+1)

        if parent is not None:
            s += std + '}\n'
        else:
            for name, node in self.nodes.iteritems():
                if name not in self.parents:
                    if self.is_valid_node(node):
                        s += node.dot_code() 
        return s

    def get_proper_group(self, topic, node):
        if node in self.group_topics and self.group_topics[node]==topic:
            return node
        elif node not in self.parents or self.parents[node] is None:
            return None
        else:
            return self.get_proper_group(topic, self.parents[node])
    
class Node:
    def __init__(self, name, is_msg):
        self.name = name
        self.badness = 0
        self.is_msg = is_msg
        self.group = None

    def get_id(self):
        if self.is_msg:
            return safe_dotcode_name(self.name) + '_msg'
        else:
            return safe_dotcode_name(self.name)

    def dot_code(self):
        s = '  %s '%self.get_id()
        m = {}
        m['label'] = self.name

        if self.is_msg:
            m['URL']   = 'topic:%s'%self.name
        else:
            m['URL']   = 'node:%s'%self.name

        if self.is_msg:
            m['shape'] = 'box'
            m['style'] = 'filled'
            m['fillcolor'] = 'white'
        elif self.badness==2:
            m['color'] = 'red'
            m['shape'] = 'doublecircle'
        elif self.badness==1:
            m['color'] = 'orange'
            m['shape'] = 'doublecircle'    
        else:
            m['style'] = 'filled'
            m['fillcolor'] = 'white'

        atts = ', '.join(['%s="%s"'%p for p in m.items()])
        return '%s[%s];'%(s,atts)

    def __repr__(self):
        return self.name

class Edge:
    def __init__(self, e):
        self.start = e.start.strip()
        self.end = e.end.strip()
        self.label = e.label.strip()

    def dot_code(self, graph):
        start = graph.nodes[self.start]
        end = graph.nodes[self.end]
        s = '    %s->%s'%(start.get_id(), end.get_id())

        sub = []
        if self.label:
            sub.append('label="%s"'%self.label)

        tail = graph.get_proper_group(self.label, self.start)
        head = graph.get_proper_group(self.label, self.end)
        if head:
            sub.append('lhead="cluster%s"'%head)
        if tail:
            sub.append('ltail="cluster%s"'%tail)
        if len(sub)>0:
            s += ' [%s]'% ','.join(sub)
        return s



def generate_dotcode(g, ns_filter, graph_mode, orientation, config, quiet=False):
    """
    @param g: Graph instance
    @param ns_filter: namespace filter (must be canonicalized with trailing '/')
    @type  ns_filter: string
    @param graph_mode str: NODE_NODE_GRAPH | NODE_TOPIC_GRAPH | NODE_TOPIC_ALL_GRAPH
    @type  graph_mode: str
    @param orientation: rankdir value (see ORIENTATIONS dict)
    @return: dotcode generated from graph singleton
    @rtype: str
    """
    graph = Graph(orientation, quiet)
    for node_name in g.nn_nodes:
        n = Node(node_name, False)
        graph.add_node(n)

    if graph_mode == NODE_TOPIC_GRAPH or \
             graph_mode == NODE_TOPIC_ALL_GRAPH:
        for node_name in g.nt_nodes:
            n = Node( node_name.strip(), True )
            graph.add_node(n)

    if graph_mode == NODE_NODE_GRAPH:
        edges = g.nn_edges
    elif graph_mode == NODE_TOPIC_GRAPH:
        edges = g.nt_edges
    else:
        edges = g.nt_all_edges

    for edge in edges:
        graph.edges.append( Edge(edge) )

    for node_name, node in g.bad_nodes.iteritems():
        if node.type == rosgraph.impl.graph.BadNode.DEAD:
            graph.nodes[node_name].badness = 2
        else:
            graph.nodes[node_name].badness = 1

    graph.set_hierarchy(config)

    s = graph.dot_code() 
    #print s
    return s

def generate_namespaces(g, graph_mode, quiet=False):
    """
    Determine the namespaces of the nodes being displayed
    """
    namespaces = []
    if graph_mode == NODE_NODE_GRAPH:
        nodes = g.nn_nodes
        #if quiet:
        #    nodes = [n for n in nodes if not n in QUIET_NAMES]
        namespaces = list(set([roslib.names.namespace(n) for n in nodes]))
            
    elif graph_mode == NODE_TOPIC_GRAPH or \
             graph_mode == NODE_TOPIC_ALL_GRAPH:
        nn_nodes = g.nn_nodes
        nt_nodes = g.nt_nodes        
        #if quiet:
        #    nn_nodes = [n for n in nn_nodes if not n in QUIET_NAMES]
        #    nt_nodes = [n for n in nt_nodes if not n in QUIET_NAMES]
        if nn_nodes or nt_nodes:
            namespaces = [roslib.names.namespace(n) for n in nn_nodes]
            # an annoyance with the rosgraph library is that it
            # prepends a space to topic names as they have to have
            # different graph node namees from nodes. we have to strip here
            namespaces.extend([roslib.names.namespace(n[1:]) for n in nt_nodes])

    return list(set(namespaces))

