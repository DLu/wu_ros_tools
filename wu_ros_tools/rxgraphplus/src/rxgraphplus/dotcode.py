# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id: dotcode.py 10318 2010-07-09 21:29:28Z kwc $

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

def _make_edge(start, end, label=None, head=None, tail=None):
    s = '    %s->%s'%(safe_dotcode_name(start), safe_dotcode_name(end))
    sub = []
    if label:
        sub.append('label="%s"'%label)
    if head:
        sub.append('lhead="cluster%s"'%head)
    if tail:
        sub.append('ltail="cluster%s"'%tail)
    if len(sub)>0:
        s += ' [%s]'% ','.join(sub)
    return s

def _generate_node_dotcode(node, badness):
    if badness==2:
        return '  %s [color="red", shape="doublecircle", label="%s", URL="node:%s"];'%(
            safe_dotcode_name(node), node, node)
    elif badness==1:
        return '  %s [color="orange", shape="doublecircle", label="%s", URL="node:%s"];'%(
            safe_dotcode_name(node), node, node, node)
    else:
        return '  %s [label="%s", URL="node:%s", style=filled fillcolor=white];'%(safe_dotcode_name(node), node, node)

QUIET_NAMES = ['/diag_agg', '/runtime_logger', '/pr2_dashboard', '/rviz', '/rosout', '/cpu_monitor', '/monitor','/hd_monitor', '/rxloggerlevel', '/clock']
def _quiet_filter(name):
    # ignore viewers
    for n in QUIET_NAMES:
        if n in name:
            return False
    return True

def _quiet_filter_edge(edge):
    for quiet_label in ['/time', '/clock', '/rosout']:
        if quiet_label == edge.label:
            return False
    return _quiet_filter(edge.start) and _quiet_filter(edge.end)
        
def generate_namespaces(g, graph_mode, quiet=False):
    """
    Determine the namespaces of the nodes being displayed
    """
    namespaces = []
    if graph_mode == NODE_NODE_GRAPH:
        nodes = g.nn_nodes
        if quiet:
            nodes = [n for n in nodes if not n in QUIET_NAMES]
        namespaces = list(set([roslib.names.namespace(n) for n in nodes]))
            
    elif graph_mode == NODE_TOPIC_GRAPH or \
             graph_mode == NODE_TOPIC_ALL_GRAPH:
        nn_nodes = g.nn_nodes
        nt_nodes = g.nt_nodes        
        if quiet:
            nn_nodes = [n for n in nn_nodes if not n in QUIET_NAMES]
            nt_nodes = [n for n in nt_nodes if not n in QUIET_NAMES]
        if nn_nodes or nt_nodes:
            namespaces = [roslib.names.namespace(n) for n in nn_nodes]
            # an annoyance with the rosgraph library is that it
            # prepends a space to topic names as they have to have
            # different graph node namees from nodes. we have to strip here
            namespaces.extend([roslib.names.namespace(n[1:]) for n in nt_nodes])

    return list(set(namespaces))

def get_groups(node, tree_node=None):
    groups = []
    if not tree_node and 'branch' in node and node['branch']:
        tree_node = node['branch']

    while tree_node:
        if tree_node['name']:
            groups.append(tree_node['name'])
        tree_node = tree_node['parent']

    return groups
        

def equal_groups(a, b):
    return len(a)!=0 and a==b

def gen(node_map, groups, names, name, depth=1):
    sub_str = ''
    for n in names:
        if n in node_map:
            if 'shouldDraw' in node_map[n]:
                sub_str += node_map[n]['code'] + '\n'
            del node_map[n]
        elif n in groups and n!='topics':
            sub_str += gen(node_map, groups, groups[n], n, depth+1)

    color = "grey%d"%(100-depth*10)

    if len(sub_str)>0:
        return 'subgraph %s {\n%s\nlabel = "%s"; style=filled; fillcolor=%s;\n}\n'%(safe_dotcode_name("cluster" + name), sub_str, name, color)
    else:
        return '  %s [shape=box,label="%s", style=filled fillcolor=%s]; '%(name, name, color)

def generate_node_code(tree_node, node_map, depth=1):
    s = ''
    std = ' '*depth
    tab = ' '*(depth+1)

    if tree_node['name']!=None:
        color = 'grey%d'%(100-(depth-1)*10)
        s += std + 'subgraph %s {\n' % safe_dotcode_name('cluster' + tree_node['name'])
        s += tab + 'label = "%s";\n'%tree_node['name']
        s += tab + 'style=filled;\n'
        s += tab + 'fillcolor=%s;\n'%color
    for value in tree_node['values']:
        if value not in node_map:
            continue
        s += tab + _generate_node_dotcode(value, node_map[value]['badness'])
    for branch in tree_node['children']:
        s += generate_node_code(branch, node_map, depth+1)
    if tree_node['name']!=None:
        s+= std + '}\n'
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
    #print "generate_dotcode", graph_mode
    if ns_filter:
        name_filter = ns_filter[:-1]
    
    # create the node definitions
    node_map = {}
    tree_root = {'parent': None, 'name': None, 'children': [], 'values': []}
    msgs = collections.defaultdict(set)
    do_filter = ns_filter and ns_filter != '/'

    for node in g.nn_nodes:
        if not do_filter or (node.startswith(ns_filter) or node == name_filter):
            node_map[node] = {'name': node, 'msg': False, 'branch': None}
            if node in g.bad_nodes:
                bn = g.bad_nodes[node]
                if bn.type == rosgraph.impl.graph.BadNode.DEAD:
                    node_map[node]['badness'] = 2
                else:
                    node_map[node]['badness'] = 1
            else:
                node_map[node]['badness'] = 0
            tree_root['values'].append(node)

    if graph_mode == NODE_TOPIC_GRAPH or \
             graph_mode == NODE_TOPIC_ALL_GRAPH:
        for node in g.nt_nodes:
            if not do_filter or (node[1:].startswith(ns_filter) or node[1:] == name_filter):
                node_map[node] = {'name': node, 'msg': True, 'branch': None}

    if quiet:
        for n in QUIET_NAMES:
            if n in node_map:
                del node_map[n]

    queue = [(group, config[group], tree_root) for group in config]
    while len(queue) > 0:
        (group, value, parent) = queue.pop(0)
        branch = {'name': group, 'values': [], 'children': [], 'parent': parent}
        parent['children'].append(branch)

        if type(value)==type([]):
            branch['values'] = value
            for node in value:
                if node in node_map:
                    node_map[node]['branch'] = branch
        else:
            for gr,v in value.iteritems():
                if gr == 'topics':
                    branch['values'] += v
                else:
                    queue.append((gr,v,branch))
    

    # create the edge definitions
    if graph_mode == NODE_NODE_GRAPH:
        edges = g.nn_edges
    elif graph_mode == NODE_TOPIC_GRAPH:
        edges = g.nt_edges
    else:
        edges = g.nt_all_edges
    if quiet:
        edges = filter(_quiet_filter_edge, edges)
    
    edges_str = ''

    for e in edges:
        if not e.start in node_map or not e.end in node_map:
            continue
        if node_map[e.start]['msg']:
            msg = e.start
            node = e.end
        elif node_map[e.end]['msg']:
            msg = e.end
            node = e.start
        else:
            continue

        msgs[msg] = msgs[msg].union(set( get_groups(node_map[node]) ))

    for msg, s in msgs.iteritems():
        if len(s)==1:
            del node_map[msg]

    outputEdges = {}

    for e in edges:
        if not e.start in node_map or not e.end in node_map:
            continue

        start = node_map[e.start]
        end = node_map[e.end]

        if start['branch']==end['branch']:
            continue

        #if 'ignore' in start['groups'] or 'ignore' in end['groups']:
        #    continue

        #if e.label in groups['ignore']:
        #    continue

        e_node = s_node = None
        if e.label:
            ni = start['branch']
            while ni:
                if e.label in ni['values']:
                    s_node = ni['name']
                ni = ni['parent']
            ni = end['branch']
            while ni:
                if e.label in ni['values']:
                    e_node = ni['name']
                ni = ni['parent']
        else:
            nis = start['branch']
            startpath = []
            # TODO
            """if 'msg' in node_map[e.start]['groups']:
                msg = e.start
                node = e.end
                for g in node_map[node]['groups']:
                    if msg.strip() in groups[g]:
                        e_node = 'cluster' + g
            elif 'msg' in node_map[e.end]['groups']:
                msg = e.end
                node = e.start
                for g in node_map[node]['groups']:
                    if msg.strip() in groups[g]:
                        s_node = 'cluster' + g"""

        key = (e.label, e_node, s_node)
        if key not in outputEdges or (not e_node and not s_node):
            edges_str += _make_edge(e.start, e.end, e.label, e_node, s_node) + '\n'
            outputEdges[key] = 1
        start['shouldDraw'] = True
        end['shouldDraw'] = True

    nodes_str = generate_node_code(tree_root, node_map)

    if len(nodes_str) == 0:
        nodes_str = '  empty;' 

    s = "digraph G {\n  rankdir=%(orientation)s;\ncompound=true; \n%(nodes_str)s\n%(edges_str)s}\n"%vars()
    #print s
    return s

