# UrbanMesh Tracking System

Low-bitrate adaptive mesh network simulation for urban two-wheeler tracking. 

## Overview

This project simulates a mesh network in an urban environment using real-world road data from OpenStreetMap. It features mobile nodes (representing two-wheelers) and fixed gateways. The system dynamically selects communication protocols (Zigbee-like, BLE-like, or Wi-Fi) based on battery levels, distance to gateways, and environmental obstacles.

## Features

- Real OSM road network fetching via Overpass API.
- Adaptive protocol switching (Zigbee/BLE/Wi-Fi).
- Dynamic power model with duty cycling and LOKA optimizations.
- Urban obstacle and fading modeling.
- Interactive HTML dashboard with Leaflet map and Chart.js analytics.
- Exportable simulation data in CSV format for research and analysis.

## Project Structure

- mesh_network_simulation.py: The core Python simulator.
- mesh_network_tracker.html: The generated interactive dashboard.
- mesh_simulation_data.csv: Exported results from the latest simulation run.

## Requirements

- Python 3.x
- networkx
- numpy
- requests
- pandas (optional for analysis)

## Usage

Run the simulator:

python mesh_network_simulation.py

After the simulation completes, it will automatically generate and open `mesh_network_tracker.html` in your default web browser.
