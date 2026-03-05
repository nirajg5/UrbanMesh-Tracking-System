"""
Low-Bitrate Adaptive Mesh Network for Urban Two-Wheeler Tracking
================================================================
Fetches real OSM road data → simulates node movement → adaptive protocol
selection (Zigbee-like / BLE-like / Wi-Fi) → exports Leaflet HTML dashboard.

Usage:
    pip install requests networkx numpy
    python mesh_network_simulation.py
"""

import requests
import json
import math
import random
import time
import os
import networkx as nx
import numpy as np
import webbrowser
from dataclasses import dataclass, field
from typing import List, Dict, Tuple, Optional
from collections import defaultdict

# ─────────────────────────────────────────────────────────────────────────────
# CONFIGURATION
# ─────────────────────────────────────────────────────────────────────────────
CITY_BBOX = {
    "name": "Katraj, Pune (Urban Simulation)",
    "south": 18.441,
    "west":  73.847,
    "north": 18.462,
    "east":  73.873,
}

SIM_CONFIG = {
    "num_nodes":        50,
    "num_gateways":      4,
    "simulation_steps": 200,
    "step_duration_s":   5,
    "speed_mps":         4.0,
    "seed":             42,
}

# ─────────────────────────────────────────────────────────────────────────────
# PROTOCOL DEFINITIONS
# ─────────────────────────────────────────────────────────────────────────────
@dataclass
class Protocol:
    name:           str
    range_m:        float   # communication range in metres
    tx_power_mW:    float   # transmit power in milliwatts
    bitrate_kbps:   float   # effective data rate
    base_loss:      float   # base packet-loss probability (0–1)
    color:          str     # for visualisation

PROTOCOLS = {
    "zigbee": Protocol("Zigbee-like", range_m=400, tx_power_mW=1.0,
                       bitrate_kbps=20,  base_loss=0.03, color="#00e5ff"),
    "ble":    Protocol("BLE-like",    range_m=150, tx_power_mW=0.5,
                       bitrate_kbps=10,  base_loss=0.05, color="#69ff47"),
    "wifi":   Protocol("Wi-Fi",       range_m=650, tx_power_mW=80.0,
                       bitrate_kbps=54000, base_loss=0.32, color="#ff4081"),
}

PACKET_SIZE_BYTES = 32   # GPS + metadata payload

# ─────────────────────────────────────────────────────────────────────────────
# OSM ROAD NETWORK FETCHER
# ─────────────────────────────────────────────────────────────────────────────
class OSMFetcher:
    """Fetch road network from OpenStreetMap via Overpass API."""

    OVERPASS_URL = "https://overpass-api.de/api/interpreter"

    def __init__(self, bbox: dict):
        self.bbox = bbox

    def fetch(self) -> nx.Graph:
        print(f"[OSM] Fetching road network for {self.bbox['name']} ...")
        query = f"""
        [out:json][timeout:30];
        (
          way["highway"~"^(primary|secondary|tertiary|residential|unclassified|trunk|motorway|living_street)$"]
              ({self.bbox['south']},{self.bbox['west']},
               {self.bbox['north']},{self.bbox['east']});
        );
        out body;
        >;
        out skel qt;
        """
        try:
            resp = requests.post(self.OVERPASS_URL, data={"data": query}, timeout=40)
            resp.raise_for_status()
            data = resp.json()
            return self._build_graph(data)
        except Exception as e:
            print(f"[OSM] Network error: {e}. Using synthetic road grid.")
            return self._synthetic_grid()

    def _build_graph(self, data: dict) -> nx.Graph:
        nodes_ll = {}
        for el in data["elements"]:
            if el["type"] == "node":
                nodes_ll[el["id"]] = (el["lat"], el["lon"])

        G = nx.Graph()
        for el in data["elements"]:
            if el["type"] == "way":
                nds = el["nodes"]
                for i in range(len(nds) - 1):
                    a, b = nds[i], nds[i + 1]
                    if a in nodes_ll and b in nodes_ll:
                        la, lo = nodes_ll[a]
                        lb, lob = nodes_ll[b]
                        G.add_node(a, lat=la, lon=lo)
                        G.add_node(b, lat=lb, lon=lob)
                        dist = haversine(la, lo, lb, lob)
                        G.add_edge(a, b, weight=dist)

        # Keep largest connected component
        if G.number_of_nodes() == 0:
            print("[OSM] Empty graph returned, using synthetic grid.")
            return self._synthetic_grid()

        largest = max(nx.connected_components(G), key=len)
        G = G.subgraph(largest).copy()
        print(f"[OSM] Road graph: {G.number_of_nodes()} nodes, "
              f"{G.number_of_edges()} edges")
        return G

    def _synthetic_grid(self) -> nx.Graph:
        """Fallback: 15×15 grid street network."""
        print("[OSM] Building synthetic 15×15 urban grid …")
        b = self.bbox
        G = nx.Graph()
        rows, cols = 15, 15
        lats = np.linspace(b["south"], b["north"], rows)
        lons = np.linspace(b["west"],  b["east"],  cols)
        for i, lat in enumerate(lats):
            for j, lon in enumerate(lons):
                nid = i * cols + j
                G.add_node(nid, lat=float(lat), lon=float(lon))
                if j > 0:
                    prev = i * cols + (j - 1)
                    G.add_edge(prev, nid,
                               weight=haversine(G.nodes[prev]["lat"],
                                                G.nodes[prev]["lon"],
                                                lat, lon))
                if i > 0:
                    prev = (i - 1) * cols + j
                    G.add_edge(prev, nid,
                               weight=haversine(G.nodes[prev]["lat"],
                                                G.nodes[prev]["lon"],
                                                lat, lon))
        return G

# ─────────────────────────────────────────────────────────────────────────────
# UTILITIES
# ─────────────────────────────────────────────────────────────────────────────
def haversine(lat1, lon1, lat2, lon2) -> float:
    """Return distance in metres between two (lat, lon) points."""
    R = 6_371_000
    φ1, φ2 = math.radians(lat1), math.radians(lat2)
    Δφ = math.radians(lat2 - lat1)
    Δλ = math.radians(lon2 - lon1)
    a = math.sin(Δφ/2)**2 + math.cos(φ1)*math.cos(φ2)*math.sin(Δλ/2)**2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

def lerp_latlon(lat1, lon1, lat2, lon2, t):
    return lat1 + (lat2 - lat1) * t, lon1 + (lon2 - lon1) * t

# ─────────────────────────────────────────────────────────────────────────────
# NODE MODEL
# ─────────────────────────────────────────────────────────────────────────────
@dataclass
class MeshNode:
    node_id:      int
    is_gateway:   bool
    road_graph:   nx.Graph
    road_nodes:   list          # list of graph node IDs

    # Position on road network
    current_edge: Tuple = field(default=None)   # (u, v)
    edge_progress: float = 0.0                  # 0..1 along edge
    path:         list  = field(default_factory=list)  # upcoming waypoints
    path_idx:     int   = 0

    lat:  float = 0.0
    lon:  float = 0.0

    # Energy model (mAh equivalent, 2000 mAh battery)
    battery_mAh:     float = 2000.0
    battery_max_mAh: float = 2000.0

    # Protocol state
    active_protocol: str = "zigbee"

    # Stats
    packets_sent:     int = 0
    packets_delivered: int = 0
    total_energy_mJ:  float = 0.0

    history: list = field(default_factory=list)   # [(lat,lon,protocol,battery%)]

    simulator_context: 'MeshNetworkSimulator' = None

    def __post_init__(self):
        self.battery_mAh = 1200.0  # Common Li-ion cell
        self.battery_max_mAh = 1200.0
        self._place_on_road()

    def _place_on_road(self):
        u, v = random.sample(self.road_nodes, 2)[:2]
        if not self.road_graph.has_edge(u, v):
            # pick any edge from u
            neighbors = list(self.road_graph.neighbors(u))
            if not neighbors:
                u = random.choice(self.road_nodes)
                neighbors = list(self.road_graph.neighbors(u))
            v = random.choice(neighbors)
        self.current_edge = (u, v)
        self.edge_progress = random.random()
        self._update_latlon()
        self._plan_path()

    def _update_latlon(self):
        u, v = self.current_edge
        lu = self.road_graph.nodes[u]["lat"]
        lnu = self.road_graph.nodes[u]["lon"]
        lv = self.road_graph.nodes[v]["lat"]
        lnv = self.road_graph.nodes[v]["lon"]
        self.lat, self.lon = lerp_latlon(lu, lnu, lv, lnv, self.edge_progress)

    def _plan_path(self, hops=6):
        """Plan a random walk path on the road network."""
        cur = self.current_edge[1]
        path = [cur]
        for _ in range(hops):
            neighbors = list(self.road_graph.neighbors(cur))
            if not neighbors:
                break
            nxt = random.choice(neighbors)
            path.append(nxt)
            cur = nxt
        self.path = path
        self.path_idx = 0

    def step(self, speed_mps: float, dt_s: float, density: float,
             num_neighbors: int):
        """Move the node and select protocol."""
        if self.is_gateway or self.is_dead():
            return

        dist_to_travel = speed_mps * dt_s * random.uniform(0.6, 1.2)
        self._move(dist_to_travel)

        # Calculate distance and blockage to nearest gateway for adaptivity
        gateways = [n for n in self.simulator_context.nodes if n.is_gateway]
        min_dist = float('inf')
        blocked = False
        for gw in gateways:
            d = haversine(self.lat, self.lon, gw.lat, gw.lon)
            if d < min_dist:
                min_dist = d
                blocked = self.simulator_context._is_blocked(self.lat, self.lon, gw.lat, gw.lon)

        self._select_protocol(num_neighbors, min_dist, blocked)
        self._drain_idle_battery(dt_s)
        self.history.append({
            "lat": round(self.lat, 7),
            "lon": round(self.lon, 7),
            "protocol": self.active_protocol,
            "battery_pct": round(self.battery_pct(), 1),
        })

    def _move(self, dist_m: float):
        u, v = self.current_edge
        edge_len = self.road_graph.edges[u, v]["weight"]
        remaining = edge_len * (1 - self.edge_progress)
        if dist_m < remaining:
            self.edge_progress += dist_m / edge_len
        else:
            # Advance to next waypoint
            if self.path_idx < len(self.path) - 1:
                self.path_idx += 1
                next_node = self.path[self.path_idx]
                prev_node = self.current_edge[1]
                if self.road_graph.has_edge(prev_node, next_node):
                    self.current_edge = (prev_node, next_node)
                else:
                    neighbors = list(self.road_graph.neighbors(prev_node))
                    if neighbors:
                        next_node = random.choice(neighbors)
                        self.current_edge = (prev_node, next_node)
                self.edge_progress = 0.0
            else:
                self._plan_path()
                self.edge_progress = 0.0
        self._update_latlon()

    def _select_protocol(self, num_neighbors: int, dist_to_gw: float, blocked: bool):
        """Adaptive protocol selection based on battery, distance and obstacles."""
        pct = self.battery_pct()
        
        # Override: Critical battery → BLE only (most efficient)
        if pct < 15:
            self.active_protocol = "ble"
            return

        # Adaptivity Logic:
        # 1. Close and clear LOS -> BLE (Efficient)
        # 2. Medium distance OR blocked -> Zigbee (Better penetration)
        # 3. Far distance -> Wi-Fi (Long range)
        
        if dist_to_gw < 250 and not blocked:
            self.active_protocol = "ble"
        elif dist_to_gw < 800 or blocked:
            self.active_protocol = "zigbee"
        else:
            self.active_protocol = "wifi"

    def _drain_idle_battery(self, dt_s: float):
        """Idle current draw: ~5 mA MCU + sensor, scaled for visible depletion."""
        idle_mA = 5.0
        # Scale factor so battery visibly depletes over 80 steps
        scale = 50
        self.battery_mAh -= (idle_mA * dt_s * scale) / 3600

    def drain_tx_battery(self, protocol_key: str, dt_s: float):
        proto = PROTOCOLS[protocol_key]
        tx_mA = proto.tx_power_mW / 3.3   # assume 3.3 V supply
        self.battery_mAh -= (tx_mA * dt_s) / 3600
        self.total_energy_mJ += proto.tx_power_mW * dt_s * 1000

    def battery_pct(self) -> float:
        return max(0, (self.battery_mAh / self.battery_max_mAh) * 100)

    def is_dead(self) -> bool:
        return self.battery_mAh <= 0

# ─────────────────────────────────────────────────────────────────────────────
# MESH NETWORK SIMULATOR
# ─────────────────────────────────────────────────────────────────────────────
class MeshNetworkSimulator:

    def __init__(self, road_graph: nx.Graph, config: dict):
        self.G      = road_graph
        self.cfg    = config
        random.seed(config["seed"])
        np.random.seed(config["seed"])

        road_node_ids = list(road_graph.nodes())
        self.nodes: List[MeshNode] = []

        # Place gateways at roughly central positions (quartiles of lat/lon)
        node_lats = [(nid, road_graph.nodes[nid]["lat"],
                      road_graph.nodes[nid]["lon"])
                     for nid in road_node_ids]
        node_lats.sort(key=lambda x: (x[1], x[2]))
        n = len(node_lats)
        gw_indices = [n // 4, n // 2, 3 * n // 4]
        gw_road_nodes = [node_lats[i][0] for i in gw_indices[:config["num_gateways"]]]

        for i, gw_nid in enumerate(gw_road_nodes):
            gw = MeshNode(i, True, road_graph, road_node_ids)
            gw.lat = road_graph.nodes[gw_nid]["lat"]
            gw.lon = road_graph.nodes[gw_nid]["lon"]
            gw.battery_mAh = 99999
            self.nodes.append(gw)

        # Mobile nodes
        for i in range(config["num_nodes"]):
            n = MeshNode(config["num_gateways"] + i, False, road_graph,
                         road_node_ids)
            n.simulator_context = self
            self.nodes.append(n)

        self.step_logs: List[dict] = []
        self.metrics   = defaultdict(list)

        # ── Defining Obstacles (Urban Blocks) ──
        # Format: (min_lat, min_lon, max_lat, max_lon)
        self.obstacles = [
            (18.445, 73.850, 18.448, 73.855), # Block 1
            (18.452, 73.860, 18.455, 73.865), # Block 2
            (18.442, 73.862, 18.445, 73.868), # Block 3
        ]

        # WiFi baseline stats (computed in parallel)
        self._wifi_stats = {"sent": 0, "delivered": 0, "energy_mJ": 0}

    # ── helpers ──────────────────────────────────────────────────────────────
    def _neighbors(self, node: MeshNode) -> List[MeshNode]:
        proto = PROTOCOLS[node.active_protocol]
        return [n for n in self.nodes if n is not node and not n.is_dead() and
                haversine(node.lat, node.lon, n.lat, n.lon) <= proto.range_m]

    def _is_blocked(self, p1_lat, p1_lon, p2_lat, p2_lon) -> bool:
        """Simple line-rectangle intersection check for obstacles."""
        for (min_lat, min_lon, max_lat, max_lon) in self.obstacles:
            # Check if either point is inside the obstacle
            if (min_lat <= p1_lat <= max_lat and min_lon <= p1_lon <= max_lon) or \
               (min_lat <= p2_lat <= max_lat and min_lon <= p2_lon <= max_lon):
                return True
            
            # Midpoint check (sufficient for small urban blocks)
            mid_lat, mid_lon = (p1_lat + p2_lat)/2, (p1_lon + p2_lon)/2
            if min_lat <= mid_lat <= max_lat and min_lon <= mid_lon <= max_lon:
                return True
        return False

    def _link_quality(self, a: MeshNode, b: MeshNode, proto_key: str,
                      density: float) -> float:
        proto = PROTOCOLS[proto_key]
        dist  = haversine(a.lat, a.lon, b.lat, b.lon)
        if dist > proto.range_m:
            return 0.0
        
        # Normalised path loss
        path_factor = (dist / proto.range_m) ** 1.2
        # Mild density interference for low-power protocols
        interference = min(0.10, density * 0.003)
        
        # Obstacle penalty
        obs_penalty = 0.25 if self._is_blocked(a.lat, a.lon, b.lat, b.lon) else 0.0
        
        loss = proto.base_loss + path_factor * 0.08 + interference + obs_penalty
        return max(0.0, min(1.0, 1 - loss))

    def _try_deliver(self, src: MeshNode, proto_key: str,
                     density: float, dt_s: float,
                     max_hops=6) -> bool:
        """BFS multi-hop delivery attempt. Returns True if packet reaches GW."""
        visited = {src.node_id}
        queue = [(src, 1.0)]
        hop   = 0

        # Direct check to gateways first
        for gw in [n for n in self.nodes if n.is_gateway]:
            dist = haversine(src.lat, src.lon, gw.lat, gw.lon)
            if dist <= PROTOCOLS[src.active_protocol].range_m:
                lq = self._link_quality(src, gw, src.active_protocol, density)
                if random.random() < lq:
                    src.drain_tx_battery(src.active_protocol, 1.0)
                    return True

        while queue and hop < max_hops:
            next_q = []
            for node, accum_lq in queue:
                max_range = max(p.range_m for p in PROTOCOLS.values())
                candidates = [n for n in self.nodes
                              if n is not node and not n.is_dead() and
                              n.node_id not in visited and
                              haversine(node.lat, node.lon, n.lat, n.lon) <= max_range]
                for nb in candidates:
                    visited.add(nb.node_id)
                    proto = node.active_protocol
                    lq = self._link_quality(node, nb, proto, density)
                    hop_lq = accum_lq * lq
                    if random.random() < hop_lq:
                        node.drain_tx_battery(proto, 0.5)
                        if nb.is_gateway:
                            return True
                        next_q.append((nb, hop_lq))
            queue = next_q
            hop  += 1
        return False

    # ── main simulation ───────────────────────────────────────────────────────
    def run(self):
        print(f"\n[SIM] Running {self.cfg['simulation_steps']} steps "
              f"with {len(self.nodes)} nodes …")
        dt  = self.cfg["step_duration_s"]
        spd = self.cfg["speed_mps"]

        for step in range(self.cfg["simulation_steps"]):
            alive_mobile = [n for n in self.nodes
                            if not n.is_gateway and not n.is_dead()]
            density = len(alive_mobile)

            # Move nodes
            for n in alive_mobile:
                nb_count = len(self._neighbors(n))
                n.step(spd, dt, density, nb_count)

            # Packet transmission
            step_sent = step_delivered = 0
            step_energy = 0.0
            links_this_step = []

            for n in alive_mobile:
                n.packets_sent += 1
                step_sent      += 1

                delivered = self._try_deliver(n, n.active_protocol,
                                              density, dt)
                if delivered:
                    n.packets_delivered += 1
                    step_delivered      += 1
                    print(f"  [MESH] Node {n.node_id:2d} -> Host: Location status sent successfully.")

                step_energy += n.total_energy_mJ

                # Record visible links for this step
                for nb in self._neighbors(n):
                    links_this_step.append({
                        "from": [n.lat, n.lon],
                        "to":   [nb.lat, nb.lon],
                        "protocol": n.active_protocol,
                    })

                # WiFi baseline (simulated in parallel, not affecting battery)
                self._wifi_stats["sent"] += 1
                # WiFi suffers high interference in dense urban (co-channel, multipath)
                wifi_interference = min(0.45, density * 0.018)
                wifi_lq = 1 - (PROTOCOLS["wifi"].base_loss + wifi_interference)
                if random.random() < wifi_lq:
                    self._wifi_stats["delivered"] += 1
                self._wifi_stats["energy_mJ"] += (
                    PROTOCOLS["wifi"].tx_power_mW * dt * 1000)

            pdr = step_delivered / step_sent if step_sent else 0
            avg_batt = (sum(n.battery_pct() for n in alive_mobile)
                        / len(alive_mobile)) if alive_mobile else 0

            self.metrics["step"].append(step)
            self.metrics["pdr"].append(round(pdr * 100, 1))
            self.metrics["alive"].append(len(alive_mobile))
            self.metrics["avg_battery"].append(round(avg_batt, 1))
            self.metrics["step_energy_J"].append(
                round(step_energy / 1000, 4))

            snapshot = {
                "step":  step,
                "nodes": [{
                    "id":       n.node_id,
                    "lat":      round(n.lat, 7),
                    "lon":      round(n.lon, 7),
                    "protocol": n.active_protocol,
                    "battery":  round(n.battery_pct(), 1),
                    "gateway":  n.is_gateway,
                    "dead":     n.is_dead(),
                } for n in self.nodes],
                "links": links_this_step[:80],  # cap for HTML size
                "pdr":   round(pdr * 100, 1),
                "avg_battery": round(avg_batt, 1),
            }
            self.step_logs.append(snapshot)

            if step % 10 == 0:
                print(f"  Step {step:3d} | PDR {pdr*100:.1f}% | "
                      f"Alive {len(alive_mobile)}/{self.cfg['num_nodes']} | "
                      f"Avg Batt {avg_batt:.1f}%")

        self._print_summary()

    def _print_summary(self):
        mobile = [n for n in self.nodes if not n.is_gateway]
        total_sent = sum(n.packets_sent      for n in mobile)
        total_dlvr = sum(n.packets_delivered for n in mobile)
        total_enrg = sum(n.total_energy_mJ   for n in mobile)

        wifi_pdr    = (self._wifi_stats["delivered"] /
                       max(1, self._wifi_stats["sent"])) * 100
        mesh_pdr    = (total_dlvr / max(1, total_sent)) * 100
        wifi_energy = self._wifi_stats["energy_mJ"] / 1000

        print("\n" + "="*58)
        print("  SIMULATION SUMMARY")
        print("="*58)
        print(f"  Total packets sent        : {total_sent}")
        print(f"  Mesh PDR                  : {mesh_pdr:.1f}%")
        print(f"  WiFi baseline PDR         : {wifi_pdr:.1f}%")
        print(f"  Mesh total energy         : {total_enrg/1000:.2f} J")
        print(f"  WiFi baseline energy      : {wifi_energy:.2f} J")
        print(f"  Energy saving (Mesh/WiFi) : "
              f"{(1 - total_enrg/(wifi_energy*1000+1))*100:.1f}%")
        print("="*58)

        self.summary = {
            "mesh_pdr":       round(mesh_pdr, 1),
            "wifi_pdr":       round(wifi_pdr, 1),
            "mesh_energy_J":  round(total_enrg / 1000, 2),
            "wifi_energy_J":  round(wifi_energy, 2),
            "total_sent":     total_sent,
            "total_delivered": total_dlvr,
        }

# ─────────────────────────────────────────────────────────────────────────────
# HTML / LEAFLET EXPORTER
# ─────────────────────────────────────────────────────────────────────────────
class LeafletExporter:

    def __init__(self, sim: MeshNetworkSimulator, bbox: dict):
        self.sim  = sim
        self.bbox = bbox

    def export(self, path: str):
        center_lat = (self.bbox["south"] + self.bbox["north"]) / 2
        center_lon = (self.bbox["west"]  + self.bbox["east"])  / 2

        # Serialize road edges (cap at 600 for HTML size)
        road_edges = []
        for u, v in list(self.sim.G.edges())[:600]:
            road_edges.append([
                [self.sim.G.nodes[u]["lat"], self.sim.G.nodes[u]["lon"]],
                [self.sim.G.nodes[v]["lat"], self.sim.G.nodes[v]["lon"]],
            ])

        steps_json    = json.dumps(self.sim.step_logs)
        metrics_json  = json.dumps(self.sim.metrics)
        summary_json  = json.dumps(self.sim.summary)
        roads_json    = json.dumps(road_edges)
        proto_colors  = json.dumps({k: v.color for k, v in PROTOCOLS.items()})
        obstacles_json = json.dumps(self.sim.obstacles)

        html = f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8"/>
<meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>Adaptive Mesh Network — Urban Two-Wheeler Tracker</title>

<!-- Leaflet -->
<link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.9.4/leaflet.min.css"/>
<script src="https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.9.4/leaflet.min.js"></script>
<!-- Chart.js -->
<script src="https://cdnjs.cloudflare.com/ajax/libs/Chart.js/4.4.0/chart.umd.min.js"></script>
<!-- Google Fonts -->
<link rel="preconnect" href="https://fonts.googleapis.com"/>
<link href="https://fonts.googleapis.com/css2?family=Space+Mono:wght@400;700&family=Syne:wght@400;700;800&display=swap" rel="stylesheet"/>

<style>
  :root {{
    --bg:       #0a0c14;
    --surface:  #12151f;
    --card:     #1a1e2e;
    --border:   #252a3d;
    --accent1:  #00e5ff;
    --accent2:  #69ff47;
    --accent3:  #ff4081;
    --accent4:  #ffd740;
    --text:     #e0e4f0;
    --muted:    #6b7194;
  }}
  * {{ box-sizing: border-box; margin: 0; padding: 0; }}
  body {{
    font-family: 'Syne', sans-serif;
    background: var(--bg);
    color: var(--text);
    min-height: 100vh;
    display: flex;
    flex-direction: column;
  }}

  /* ── HEADER ── */
  header {{
    display: flex;
    align-items: center;
    justify-content: space-between;
    padding: 14px 28px;
    background: var(--surface);
    border-bottom: 1px solid var(--border);
    gap: 16px;
    flex-wrap: wrap;
  }}
  .logo {{
    display: flex; align-items: center; gap: 10px;
    font-size: 1.05rem; font-weight: 800; letter-spacing: -0.02em;
  }}
  .logo-dot {{
    width: 10px; height: 10px; border-radius: 50%;
    background: var(--accent1);
    box-shadow: 0 0 10px var(--accent1);
    animation: pulse 1.5s infinite;
  }}
  @keyframes pulse {{
    0%,100% {{ opacity:1; transform:scale(1); }}
    50% {{ opacity:.5; transform:scale(1.4); }}
  }}
  .badge {{
    font-family: 'Space Mono', monospace;
    font-size: .65rem; font-weight: 700;
    padding: 3px 8px; border-radius: 4px;
    background: var(--border); color: var(--accent4);
    letter-spacing: .1em;
  }}
  .header-stats {{
    display: flex; gap: 18px; flex-wrap: wrap;
  }}
  .hstat {{
    text-align: right;
    font-family: 'Space Mono', monospace;
  }}
  .hstat .val {{
    font-size: 1.4rem; font-weight: 700; color: var(--accent1);
    line-height: 1;
  }}
  .hstat .lbl {{
    font-size: .6rem; color: var(--muted); text-transform: uppercase;
    letter-spacing: .1em;
  }}

  /* ── MAIN LAYOUT ── */
  main {{
    display: grid;
    grid-template-columns: 1fr 360px;
    grid-template-rows: 1fr auto;
    flex: 1;
    gap: 0;
  }}

  /* ── MAP ── */
  #map {{
    height: calc(100vh - 60px);
    min-height: 500px;
    background: #080b12;
    grid-row: 1 / 3;
  }}

  /* ── SIDE PANEL ── */
  .panel {{
    display: flex; flex-direction: column;
    background: var(--surface);
    border-left: 1px solid var(--border);
    overflow-y: auto;
    max-height: calc(100vh - 60px);
  }}

  /* ── CONTROLS ── */
  .ctrl-bar {{
    padding: 14px 16px;
    border-bottom: 1px solid var(--border);
    display: flex; flex-direction: column; gap: 12px;
  }}
  .ctrl-row {{ display: flex; align-items: center; gap: 10px; }}
  .btn {{
    font-family: 'Space Mono', monospace;
    font-size: .7rem; font-weight: 700;
    padding: 7px 14px; border-radius: 6px;
    border: 1px solid var(--accent1);
    color: var(--accent1); background: transparent;
    cursor: pointer; letter-spacing: .05em;
    transition: all .15s;
  }}
  .btn:hover, .btn.active {{
    background: var(--accent1); color: var(--bg);
  }}
  .btn.danger {{ border-color: var(--accent3); color: var(--accent3); }}
  .btn.danger:hover {{ background: var(--accent3); color: #fff; }}
  input[type=range] {{
    flex: 1; accent-color: var(--accent1);
    background: var(--border); border-radius: 4px; height: 4px;
  }}
  .ctrl-label {{
    font-family: 'Space Mono', monospace;
    font-size: .65rem; color: var(--muted); white-space: nowrap;
  }}
  #step-display {{
    font-family: 'Space Mono', monospace;
    font-size: .75rem; color: var(--accent4);
    min-width: 55px; text-align: right;
  }}

  /* ── PROTOCOL LEGEND ── */
  .legend {{
    padding: 12px 16px;
    border-bottom: 1px solid var(--border);
  }}
  .legend-title {{
    font-size: .65rem; color: var(--muted);
    text-transform: uppercase; letter-spacing: .12em;
    margin-bottom: 8px;
  }}
  .legend-row {{
    display: flex; align-items: center; gap: 8px;
    margin-bottom: 6px; font-size: .75rem;
  }}
  .legend-dot {{
    width: 10px; height: 10px; border-radius: 50%; flex-shrink: 0;
  }}
  .legend-sub {{
    font-family: 'Space Mono', monospace;
    font-size: .6rem; color: var(--muted); margin-left: auto;
  }}

  /* ── METRIC CARDS ── */
  .cards {{
    display: grid; grid-template-columns: 1fr 1fr;
    gap: 8px; padding: 12px 16px;
    border-bottom: 1px solid var(--border);
  }}
  .card {{
    background: var(--card); border-radius: 8px;
    padding: 10px 12px; border: 1px solid var(--border);
  }}
  .card .val {{
    font-family: 'Space Mono', monospace;
    font-size: 1.3rem; font-weight: 700;
    color: var(--accent1); line-height: 1;
  }}
  .card .lbl {{
    font-size: .6rem; color: var(--muted);
    text-transform: uppercase; letter-spacing: .08em;
    margin-top: 3px;
  }}
  .card.green .val {{ color: var(--accent2); }}
  .card.red   .val {{ color: var(--accent3); }}
  .card.gold  .val {{ color: var(--accent4); }}

  /* ── CHARTS ── */
  .chart-wrap {{
    padding: 12px 16px;
    border-bottom: 1px solid var(--border);
  }}
  .chart-title {{
    font-size: .65rem; color: var(--muted);
    text-transform: uppercase; letter-spacing: .12em;
    margin-bottom: 8px;
  }}
  canvas {{ max-width: 100%; }}

  /* ── COMPARISON TABLE ── */
  .compare {{
    padding: 12px 16px;
  }}
  .compare table {{
    width: 100%; border-collapse: collapse;
    font-size: .72rem; font-family: 'Space Mono', monospace;
  }}
  .compare th {{
    color: var(--muted); font-weight: 400;
    text-transform: uppercase; font-size: .58rem;
    padding: 4px 6px; border-bottom: 1px solid var(--border);
    text-align: left; letter-spacing: .08em;
  }}
  .compare td {{
    padding: 5px 6px; border-bottom: 1px solid var(--border);
  }}
  .compare .good {{ color: var(--accent2); }}
  .compare .bad  {{ color: var(--accent3); }}

  /* ── SCROLLBAR ── */
  .panel::-webkit-scrollbar {{ width: 4px; }}
  .panel::-webkit-scrollbar-track {{ background: var(--surface); }}
  .panel::-webkit-scrollbar-thumb {{
    background: var(--border); border-radius: 2px;
  }}

  /* ── LEAFLET DARK ── */
  .leaflet-layer, .leaflet-control-zoom, .leaflet-control-attribution {{
    filter: invert(1) hue-rotate(180deg) brightness(.7) saturate(1.5);
  }}
  .leaflet-container {{ background: #080b12 !important; }}
</style>
</head>
<body>

<header>
  <div class="logo">
    <div class="logo-dot"></div>
    <div>
      Adaptive Mesh Network
      <br/><span style="font-size:.75rem;color:var(--muted);font-weight:400;">
        Urban Two-Wheeler Tracker · {self.bbox['name']}
      </span>
    </div>
    <span class="badge">OSM + OVERPASS</span>
  </div>
  <div class="header-stats">
    <div class="hstat">
      <div class="val" id="hdr-pdr">—</div>
      <div class="lbl">Mesh PDR</div>
    </div>
    <div class="hstat">
      <div class="val" id="hdr-alive" style="color:var(--accent2)">—</div>
      <div class="lbl">Alive Nodes</div>
    </div>
    <div class="hstat">
      <div class="val" id="hdr-batt" style="color:var(--accent4)">—</div>
      <div class="lbl">Avg Battery</div>
    </div>
  </div>
</header>

<main>
  <div id="map"></div>

  <aside class="panel">
    <!-- controls -->
    <div class="ctrl-bar">
      <div class="ctrl-row">
        <button class="btn" id="btn-play" onclick="togglePlay()">▶ PLAY</button>
        <button class="btn" id="btn-step" onclick="stepOnce()">STEP</button>
        <button class="btn danger" onclick="reset()">RESET</button>
      </div>
      <div class="ctrl-row">
        <span class="ctrl-label">Speed</span>
        <input type="range" id="speed-sl" min="50" max="800" value="200"
               style="flex:1"/>
        <span class="ctrl-label" id="speed-lbl">200ms</span>
      </div>
      <div class="ctrl-row">
        <span class="ctrl-label">Step</span>
        <input type="range" id="step-sl" min="0"
               max="{len(self.sim.step_logs)-1}" value="0"
               oninput="seekTo(+this.value)" style="flex:1"/>
        <span id="step-display">0 / {len(self.sim.step_logs)-1}</span>
      </div>
      <div class="ctrl-row" style="gap:6px">
        <label style="font-size:.65rem;color:var(--muted);">
          <input type="checkbox" id="chk-roads" checked onchange="toggleRoads()"/>
          Roads
        </label>
        <label style="font-size:.65rem;color:var(--muted);">
          <input type="checkbox" id="chk-links" checked onchange="toggleLinks()"/>
          Mesh Links
        </label>
        <label style="font-size:.65rem;color:var(--muted);">
          <input type="checkbox" id="chk-trails" onchange="toggleTrails()"/>
          Trails
        </label>
      </div>
    </div>

    <!-- legend -->
    <div class="legend">
      <div class="legend-title">Protocol Legend</div>
      <div class="legend-row">
        <div class="legend-dot" style="background:#00e5ff;box-shadow:0 0 6px #00e5ff"></div>
        <span>Zigbee-like</span>
        <span class="legend-sub">120m · 1mW · 20kbps</span>
      </div>
      <div class="legend-row">
        <div class="legend-dot" style="background:#69ff47;box-shadow:0 0 6px #69ff47"></div>
        <span>BLE-like</span>
        <span class="legend-sub">60m · 0.5mW · 10kbps</span>
      </div>
      <div class="legend-row">
        <div class="legend-dot" style="background:#ff4081;box-shadow:0 0 6px #ff4081"></div>
        <span>Wi-Fi (Baseline)</span>
        <span class="legend-sub">200m · 80mW · 54Mbps</span>
      </div>
      <div class="legend-row">
        <div class="legend-dot" style="background:#ffd740;box-shadow:0 0 6px #ffd740;border-radius:3px"></div>
        <span>Gateway (Fixed)</span>
        <span class="legend-sub">Always-on relay</span>
      </div>
      <div class="legend-row">
        <div class="legend-dot" style="background:#555;border:1px solid #888"></div>
        <span>Dead node</span>
        <span class="legend-sub">Battery depleted</span>
      </div>
    </div>

    <!-- metric cards -->
    <div class="cards">
      <div class="card">
        <div class="val" id="c-pdr">—</div>
        <div class="lbl">PDR %</div>
      </div>
      <div class="card green">
        <div class="val" id="c-alive">—</div>
        <div class="lbl">Alive Nodes</div>
      </div>
      <div class="card gold">
        <div class="val" id="c-batt">—</div>
        <div class="lbl">Avg Battery</div>
      </div>
      <div class="card red">
        <div class="val" id="c-energy">—</div>
        <div class="lbl">Step Energy J</div>
      </div>
    </div>

    <!-- PDR chart -->
    <div class="chart-wrap">
      <div class="chart-title">Packet Delivery Ratio over Time</div>
      <canvas id="chart-pdr" height="110"></canvas>
    </div>

    <!-- Battery chart -->
    <div class="chart-wrap">
      <div class="chart-title">Average Battery Level</div>
      <canvas id="chart-batt" height="100"></canvas>
    </div>

    <!-- Comparison -->
    <div class="compare">
      <div class="chart-title">Mesh vs Wi-Fi Comparison</div>
      <table>
        <tr>
          <th>Metric</th><th>Mesh</th><th>Wi-Fi</th>
        </tr>
        <tr>
          <td>PDR</td>
          <td class="good">{self.sim.summary['mesh_pdr']}%</td>
          <td class="bad">{self.sim.summary['wifi_pdr']}%</td>
        </tr>
        <tr>
          <td>Energy (J)</td>
          <td class="good">{self.sim.summary['mesh_energy_J']}</td>
          <td class="bad">{self.sim.summary['wifi_energy_J']}</td>
        </tr>
        <tr>
          <td>Pkts Sent</td>
          <td colspan="2" style="color:var(--text)">{self.sim.summary['total_sent']}</td>
        </tr>
        <tr>
          <td>Delivered</td>
          <td colspan="2" class="good">{self.sim.summary['total_delivered']}</td>
        </tr>
      </table>
    </div>
  </aside>
</main>

<script>
// ── DATA ─────────────────────────────────────────────────────────────────────
const STEPS        = {steps_json};
const METRICS      = {metrics_json};
const ROAD_EDGES   = {roads_json};
const PROTO_COLORS = {proto_colors};
const OBSTACLES    = {obstacles_json};
const CENTER       = [{center_lat}, {center_lon}];
const TOTAL_STEPS  = STEPS.length;

// ── MAP INIT ──────────────────────────────────────────────────────────────────
const map = L.map('map', {{ zoomControl: true }}).setView(CENTER, 15);
L.tileLayer('https://{{s}}.tile.openstreetmap.org/{{z}}/{{x}}/{{y}}.png', {{
  attribution: '© OpenStreetMap contributors',
  maxZoom: 19,
}}).addTo(map);

// ── ROAD LAYER ───────────────────────────────────────────────────────────────
const roadLayer = L.layerGroup();
ROAD_EDGES.forEach(([a, b]) => {{
  L.polyline([a, b], {{
    color: '#1e2d42', weight: 2.5, opacity: .7,
  }}).addTo(roadLayer);
}});
roadLayer.addTo(map);

// ── OBSTACLE LAYER ───────────────────────────────────────────────────────────
OBSTACLES.forEach(([minLat, minLon, maxLat, maxLon]) => {{
  L.rectangle([[minLat, minLon], [maxLat, maxLon]], {{
    color: '#333', weight: 1, fillOpacity: 0.6, fillColor: '#1a1e2e'
  }}).addTo(map)
     .bindTooltip('Urban Obstacle', {{ permanent: false, direction: 'center' }});
}});

// ── STATE ─────────────────────────────────────────────────────────────────────
let currentStep = 0;
let playing     = false;
let playTimer   = null;
let showLinks   = true;
let showTrails  = false;

const nodeMarkers  = {{}};
const linkLines    = [];
const trailPaths   = {{}};  // nodeId → [latlons]
const nodeHistory  = {{}}; // nodeId → [latlons]

// Pre-build history per node
STEPS.forEach(s => {{
  s.nodes.forEach(n => {{
    if (!nodeHistory[n.id]) nodeHistory[n.id] = [];
    nodeHistory[n.id].push([n.lat, n.lon]);
  }});
}});

// ── NODE ICONS ────────────────────────────────────────────────────────────────
function makeIcon(color, size, shape, dead) {{
  const svg = dead
    ? `<svg xmlns="http://www.w3.org/2000/svg" width="${{size}}" height="${{size}}" viewBox="0 0 24 24"><circle cx="12" cy="12" r="10" fill="#333" stroke="#666" stroke-width="2"/><line x1="7" y1="7" x2="17" y2="17" stroke="#888" stroke-width="2.5"/><line x1="17" y1="7" x2="7" y2="17" stroke="#888" stroke-width="2.5"/></svg>`
    : shape === 'gw'
      ? `<svg xmlns="http://www.w3.org/2000/svg" width="${{size}}" height="${{size}}" viewBox="0 0 24 24"><rect x="3" y="3" width="18" height="18" rx="3" fill="${{color}}" opacity=".9"/><rect x="7" y="7" width="10" height="10" rx="1" fill="rgba(0,0,0,0.5)"/></svg>`
      : `<svg xmlns="http://www.w3.org/2000/svg" width="${{size}}" height="${{size}}" viewBox="0 0 24 24"><circle cx="12" cy="12" r="10" fill="${{color}}" opacity=".9"/><circle cx="12" cy="12" r="4" fill="rgba(0,0,0,0.4)"/></svg>`;
  return L.divIcon({{
    html: svg,
    className: '',
    iconSize:   [size, size],
    iconAnchor: [size/2, size/2],
  }});
}}

// ── RENDER STEP ───────────────────────────────────────────────────────────────
function renderStep(step) {{
  const data = STEPS[step];

  // Clear links
  linkLines.forEach(l => map.removeLayer(l));
  linkLines.length = 0;

  // Nodes
    data.nodes.forEach(n => {{
    const color = n.gateway ? '#ffd740'
                : n.dead    ? '#555'
                : (PROTO_COLORS[n.protocol] || '#00e5ff');
    const size  = n.gateway ? 30 : 22;
    const shape = n.gateway ? 'gw' : 'node';
    const icon  = makeIcon(color, size, shape, n.dead);

    if (nodeMarkers[n.id]) {{
      nodeMarkers[n.id].setLatLng([n.lat, n.lon]);
      nodeMarkers[n.id].setIcon(icon);
      nodeMarkers[n.id].setTooltipContent(
        `<b>${{n.gateway ? '🏠 Gateway' : '🛵 Node ' + n.id}}</b><br/>
         Protocol: ${{n.protocol}}<br/>
         Battery: ${{n.battery}}%<br/>
         Status: ${{n.dead ? '💀 Dead' : '✅ Active'}}`
      );
    }} else {{
      nodeMarkers[n.id] = L.marker([n.lat, n.lon], {{ icon }})
        .bindTooltip(
          `<b>${{n.gateway ? '🏠 Gateway' : '🛵 Node ' + n.id}}</b><br/>
           Protocol: ${{n.protocol}}<br/>
           Battery: ${{n.battery}}%`,
          {{ permanent: false, direction: 'top' }}
        )
        .addTo(map);
    }}
  }});

  // Trails
  if (showTrails) {{
    data.nodes.forEach(n => {{
      if (n.gateway) return;
      const hist = nodeHistory[n.id].slice(0, step + 1);
      const color = PROTO_COLORS[n.protocol] || '#00e5ff';
      if (trailPaths[n.id]) {{
        map.removeLayer(trailPaths[n.id]);
      }}
      trailPaths[n.id] = L.polyline(hist, {{
        color, weight: 1.5, opacity: .4, dashArray: '3 4',
      }}).addTo(map);
    }});
  }}

  // Links
  if (showLinks) {{
    data.links.forEach(lk => {{
      const c = PROTO_COLORS[lk.protocol] || '#00e5ff';
      const l = L.polyline([lk.from, lk.to], {{
        color: c, weight: 1.2, opacity: .45, dashArray: '4 6',
      }}).addTo(map);
      linkLines.push(l);
    }});
  }}

  // Metric cards
  document.getElementById('c-pdr').textContent    = data.pdr + '%';
  document.getElementById('c-alive').textContent  = data.nodes.filter(n => !n.gateway && !n.dead).length;
  document.getElementById('c-batt').textContent   = data.avg_battery + '%';
  document.getElementById('c-energy').textContent = (METRICS.step_energy_J[step] || 0).toFixed(3);

  // Header
  document.getElementById('hdr-pdr').textContent   = data.pdr + '%';
  document.getElementById('hdr-alive').textContent  = data.nodes.filter(n => !n.gateway && !n.dead).length;
  document.getElementById('hdr-batt').textContent   = data.avg_battery + '%';

  // Slider
  document.getElementById('step-sl').value = step;
  document.getElementById('step-display').textContent = step + ' / ' + (TOTAL_STEPS - 1);

  // Charts highlight
  if (window.pdrChart) {{
    pdrChart.data.datasets[0].pointRadius = METRICS.step.map((_, i) => i === step ? 5 : 0);
    pdrChart.update('none');
  }}
}}

// ── CHARTS ────────────────────────────────────────────────────────────────────
const chartDefaults = {{
  responsive: true,
  animation: false,
  plugins: {{ legend: {{ display: false }} }},
  scales: {{
    x: {{ ticks: {{ color: '#6b7194', font: {{ size: 9 }} }},
          grid: {{ color: '#252a3d' }} }},
    y: {{ ticks: {{ color: '#6b7194', font: {{ size: 9 }} }},
          grid: {{ color: '#252a3d' }} }},
  }},
}};

const pdrChart = new Chart(document.getElementById('chart-pdr'), {{
  type: 'line',
  data: {{
    labels: METRICS.step,
    datasets: [{{
      data: METRICS.pdr,
      borderColor: '#00e5ff',
      backgroundColor: 'rgba(0,229,255,.08)',
      fill: true,
      pointRadius: 0,
      borderWidth: 1.8,
      tension: 0.4,
    }}],
  }},
  options: {{ ...chartDefaults,
    scales: {{ ...chartDefaults.scales,
      y: {{ ...chartDefaults.scales.y, min: 0, max: 100 }}
    }}
  }},
}});

const battChart = new Chart(document.getElementById('chart-batt'), {{
  type: 'line',
  data: {{
    labels: METRICS.step,
    datasets: [{{
      data: METRICS.avg_battery,
      borderColor: '#ffd740',
      backgroundColor: 'rgba(255,215,64,.08)',
      fill: true,
      pointRadius: 0,
      borderWidth: 1.8,
      tension: 0.4,
    }}],
  }},
  options: {{ ...chartDefaults,
    scales: {{ ...chartDefaults.scales,
      y: {{ ...chartDefaults.scales.y, min: 0, max: 100 }}
    }}
  }},
}});

// ── CONTROLS ──────────────────────────────────────────────────────────────────
function togglePlay() {{
  playing = !playing;
  document.getElementById('btn-play').textContent = playing ? '⏸ PAUSE' : '▶ PLAY';
  document.getElementById('btn-play').classList.toggle('active', playing);
  if (playing) scheduleNext();
  else clearTimeout(playTimer);
}}
function scheduleNext() {{
  if (!playing) return;
  const delay = +document.getElementById('speed-sl').value;
  document.getElementById('speed-lbl').textContent = delay + 'ms';
  playTimer = setTimeout(() => {{
    if (currentStep < TOTAL_STEPS - 1) {{
      currentStep++;
      renderStep(currentStep);
      scheduleNext();
    }} else {{
      playing = false;
      document.getElementById('btn-play').textContent = '▶ PLAY';
      document.getElementById('btn-play').classList.remove('active');
    }}
  }}, delay);
}}
function stepOnce() {{
  if (currentStep < TOTAL_STEPS - 1) {{ currentStep++; renderStep(currentStep); }}
}}
function seekTo(s) {{
  currentStep = s;
  renderStep(s);
}}
function reset() {{
  clearTimeout(playTimer);
  playing = false;
  document.getElementById('btn-play').textContent = '▶ PLAY';
  document.getElementById('btn-play').classList.remove('active');
  currentStep = 0;
  renderStep(0);
}}
function toggleRoads() {{
  if (document.getElementById('chk-roads').checked) roadLayer.addTo(map);
  else map.removeLayer(roadLayer);
}}
function toggleLinks() {{
  showLinks = document.getElementById('chk-links').checked;
  renderStep(currentStep);
}}
function toggleTrails() {{
  showTrails = document.getElementById('chk-trails').checked;
  if (!showTrails) {{
    Object.values(trailPaths).forEach(l => map.removeLayer(l));
    Object.keys(trailPaths).forEach(k => delete trailPaths[k]);
  }}
  renderStep(currentStep);
}}
document.getElementById('speed-sl').addEventListener('input', function() {{
  document.getElementById('speed-lbl').textContent = this.value + 'ms';
}});

// ── BOOT ─────────────────────────────────────────────────────────────────────
renderStep(0);
</script>
</body>
</html>"""

        with open(path, "w", encoding="utf-8") as f:
            f.write(html)
        print(f"\n[HTML] Exported -> {path}  ({os.path.getsize(path)//1024} KB)")

# ─────────────────────────────────────────────────────────────────────────────
# ENTRY POINT
# ─────────────────────────────────────────────────────────────────────────────
def main():
    print("=" * 58)
    print("  Low-Bitrate Adaptive Mesh Network Simulator")
    print("  Urban Two-Wheeler Tracking · OSM + Overpass")
    print("=" * 58)

    # 1. Fetch road network
    fetcher = OSMFetcher(CITY_BBOX)
    road_graph = fetcher.fetch()

    # 2. Run simulation
    sim = MeshNetworkSimulator(road_graph, SIM_CONFIG)
    sim.run()

    # 3. Export HTML
    out_path = os.path.abspath("mesh_network_tracker.html")
    exporter = LeafletExporter(sim, CITY_BBOX)
    exporter.export(out_path)

    print(f"\n[OK] Done! Opening {out_path} in your browser...")
    webbrowser.open(f"file://{out_path}")

if __name__ == "__main__":
    main()
