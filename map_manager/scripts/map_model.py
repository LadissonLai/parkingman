import xml.etree.ElementTree as ET
import numpy as np

class Node:
    """ 
    存储路网中的离散点信息 
    Coordinate System: Right-handed
    Units: Meters, Degrees (-180 to 180)
    """
    def __init__(self, nid, p_str):
        self.id = int(nid)
        # 解析 "x y z theta_deg"
        coords = [float(val) for val in p_str.split()]
        self.x = coords[0]
        self.y = coords[1]
        self.z = coords[2]
        self.theta_deg = coords[3] # 角度制存储

    @property
    def position(self):
        return np.array([self.x, self.y, self.z])
    
    @property
    def theta_rad(self):
        """ 辅助属性：转弧度用于计算 """
        return np.radians(self.theta_deg)

class Way:
    def __init__(self, wid, node_ids_str, w_type):
        self.id = int(wid)
        self.node_ids = [int(nid) for nid in node_ids_str.split()]
        self.type = w_type 

class RoadMap:
    def __init__(self):
        self.nodes = {} 
        self.ways = {}        
        self.adjacency = {} 

    def load_from_string(self, xml_content):
        root = ET.fromstring(xml_content)
        self._parse_tree(ET.ElementTree(root))

    def load_from_file(self, filename):
        tree = ET.parse(filename)
        self._parse_tree(tree)

    def _parse_tree(self, tree):
        root = tree.getroot()
        for n in root.find('nodes'):
            node = Node(n.get('id'), n.get('p'))
            self.nodes[node.id] = node

        for w in root.find('ways'):
            way = Way(w.get('id'), w.get('nodes'), w.get('type'))
            self.ways[way.id] = way
            if way.id not in self.adjacency:
                self.adjacency[way.id] = []

        for r in root.find('relations'):
            u = int(r.get('from'))
            v = int(r.get('to'))
            if u in self.adjacency:
                self.adjacency[u].append(v)
            else:
                self.adjacency[u] = [v]
        print(f"[Map] Loaded {len(self.nodes)} nodes, {len(self.ways)} ways.")