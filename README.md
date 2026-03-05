# 🛵 UrbanMesh Tracking System

> **Low-Bitrate Adaptive Mesh Network for Urban Two-Wheeler Tracking**

A simulation of a self-organizing, adaptive wireless mesh network that tracks two-wheelers (bikes/scooters) in real urban environments. The system fetches live road data from OpenStreetMap (Katraj, Pune), simulates 50 mobile nodes moving along real streets, and dynamically selects the optimal radio protocol (Zigbee-like / BLE / Wi-Fi) based on distance, battery level, and urban obstacles.

---

## 📸 Dashboard Preview

The simulation exports a fully interactive **Leaflet.js HTML dashboard** (`mesh_network_tracker.html`) with:
- Real-time animated node movement on OSM map tiles
- Live protocol color-coding per node
- Battery drain charts & PDR (Packet Delivery Ratio) metrics
- Urban obstacle visualization
- Step-by-step playback slider

---

## 🌐 Area of Simulation

| Parameter | Value |
|-----------|-------|
| Location | Katraj, Pune, India |
| Bounding Box | 18.441°N – 18.462°N, 73.847°E – 73.873°E |
| Road Data Source | OpenStreetMap via Overpass API |

---

## ⚙️ System Architecture

```
OSM Road Network (Overpass API)
        ↓
  Road Graph (NetworkX)
        ↓
  50 Mobile Nodes + 4 Gateways
        ↓
  Adaptive Protocol Selection
   ┌──────────────────────────┐
   │ BLE      (<250m, clear)  │
   │ Zigbee   (<800m / blocked)│
   │ Wi-Fi    (>800m, open)   │
   └──────────────────────────┘
        ↓
  Multi-hop BFS Packet Delivery
        ↓
  Leaflet HTML Dashboard Export
```

---

## 📡 Supported Protocols

| Protocol | Range | TX Power | Bitrate | Best For |
|----------|-------|----------|---------|----------|
| **BLE-like** | 150 m | 0.5 mW | 10 Kbps | Short range, energy-critical nodes |
| **Zigbee-like** | 400 m | 1.0 mW | 20 Kbps | Medium range, obstacle-dense urban areas |
| **Wi-Fi** | 650 m | 80.0 mW | 54 Mbps | Long range, when battery permits |

Adaptive selection logic:
- 🔋 **Battery < 15%** → Force BLE (most efficient)
- 📍 **Distance < 250m + clear LOS** → BLE
- 🏙️ **Distance < 800m or blocked** → Zigbee
- 🌐 **Distance > 800m** → Wi-Fi

---

## 🚀 Getting Started

### Prerequisites

```bash
pip install requests networkx numpy
```

Python 3.8+ is required.

### Run the Simulation

```bash
python mesh_network_simulation.py
```

This will:
1. Fetch the real road network from OpenStreetMap (or fall back to a synthetic 15×15 grid)
2. Simulate **200 time steps** (5 s each → 1000 s total)
3. Print per-step stats to console
4. Export `mesh_network_tracker.html` to the current directory
5. Auto-open the dashboard in your browser

### Open the Dashboard (manually)

```bash
# Just open the exported file in any browser:
mesh_network_tracker.html
```

---

## 📊 Simulation Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `num_nodes` | 50 | Number of mobile tracking nodes |
| `num_gateways` | 4 | Fixed base-station gateways |
| `simulation_steps` | 200 | Total simulation steps |
| `step_duration_s` | 5 | Seconds per step |
| `speed_mps` | 4.0 | Average node speed (m/s) |
| `seed` | 42 | Random seed for reproducibility |

Edit `SIM_CONFIG` in `mesh_network_simulation.py` to change these.

---

## 📁 Project Structure

```
UrbanMesh-Tracking-System/
├── mesh_network_simulation.py   # Main simulation engine
├── mesh_network_tracker.html    # Pre-generated interactive dashboard
└── README.md                    # This file
```

---

## 🧮 Key Metrics

The simulation computes and displays:

- **PDR** (Packet Delivery Ratio) — % of packets reaching a gateway
- **Alive nodes** — nodes with remaining battery
- **Average battery %** — fleet-wide energy state
- **Energy usage (J)** — mesh vs. WiFi-only baseline comparison
- **Energy savings** — how much more efficient the adaptive mesh is vs. pure Wi-Fi

---

## 🏙️ Urban Obstacle Modeling

Three rectangular urban building blocks are modeled in Katraj:

| Block | Bounds |
|-------|--------|
| Block 1 | 18.445–18.448°N, 73.850–73.855°E |
| Block 2 | 18.452–18.455°N, 73.860–73.865°E |
| Block 3 | 18.442–18.445°N, 73.862–73.868°E |

Obstacles increase packet loss probability and trigger protocol upgrades (e.g., BLE → Zigbee) for better signal penetration.

---

## 🔋 Energy Model

- Battery: 1200 mAh Li-ion per node
- Idle drain: 5 mA MCU + sensor (scaled for visible simulation depletion)
- TX drain: based on protocol TX power at 3.3V supply
- Gateways: effectively unlimited (99999 mAh)

---

## 📜 License

This project is open-source for educational and research purposes.

---

## 🙏 Acknowledgements

- [OpenStreetMap](https://www.openstreetmap.org/) — Road data via Overpass API
- [NetworkX](https://networkx.org/) — Graph-based road network modeling
- [Leaflet.js](https://leafletjs.com/) — Interactive map dashboard
- [Chart.js](https://www.chartjs.org/) — Real-time metric charts
