"""
CanSat AlphaSpot - Blender 3D Flight Path Simulation

This script creates a 3D animation of the CanSat flight path using Blender.
It reads GPS coordinates from a data file and creates keyframe animations
showing the precise flight trajectory in 3D space.

Usage:
    Run this script from within Blender:
    1. Open Blender
    2. Open the script editor
    3. Run this script
    4. Ensure data file path is correct

Requirements:
    - Blender 2.8+ installed
    - GPS data file (data.txt format: longitude,latitude,altitude)
    - Object named 'Cube.001' in Blender scene (representing the CanSat)
"""

import bpy
import math
import os
from pathlib import Path
from typing import List, Tuple, Optional


def parse_gps_data(data_file: str) -> List[Tuple[float, float, float]]:
    """
    Parse GPS data from file.
    
    Args:
        data_file: Path to GPS data file
        
    Returns:
        List of tuples containing (longitude, latitude, altitude)
    """
    coordinates = []
    
    try:
        with open(data_file, 'r') as file:
            content = file.read()
            lines = content.split("\n")
            
            for line in lines:
                if line.strip():  # Skip empty lines
                    parts = line.split(",")
                    if len(parts) >= 3:
                        try:
                            longitude = float(parts[0])
                            latitude = float(parts[1])
                            altitude = float(parts[2])
                            coordinates.append((longitude, latitude, altitude))
                        except ValueError:
                            print(f"Skipping invalid line: {line}")
                            
    except FileNotFoundError:
        print(f"Error: Data file not found: {data_file}")
        return []
    except Exception as e:
        print(f"Error reading data file: {e}")
        return []
    
    return coordinates


def convert_to_3d_coords(
    current: Tuple[float, float, float],
    previous: Optional[Tuple[float, float, float]] = None
) -> Tuple[float, float, float]:
    """
    Convert GPS coordinates to 3D Cartesian coordinates.
    
    Args:
        current: Current GPS coordinates (lon, lat, alt)
        previous: Previous GPS coordinates (for relative positioning)
        
    Returns:
        3D coordinates (x, y, z) in meters
    """
    EARTH_RADIUS = 6371000  # Earth radius in meters
    
    if previous is None:
        return (0.0, 0.0, 0.0)
    
    # Calculate differences
    lon_diff = current[0] - previous[0]
    lat_diff = current[1] - previous[1]
    altitude = current[2]
    
    # Convert to meters
    # Longitude: x-axis (east-west)
    x_axis = 2 * math.pi * EARTH_RADIUS * (lon_diff / 360.0)
    
    # Latitude: y-axis (north-south)
    y_axis = 2 * math.pi * EARTH_RADIUS * (lat_diff / 360.0)
    
    # Altitude: z-axis (up-down)
    z_axis = altitude
    
    return (x_axis, y_axis, z_axis)


def create_animation(data_file: str, object_name: str = "Cube.001") -> bool:
    """
    Create 3D animation of flight path in Blender.
    
    Args:
        data_file: Path to GPS data file
        object_name: Name of Blender object to animate
        
    Returns:
        True if successful, False otherwise
    """
    # Parse GPS data
    coordinates = parse_gps_data(data_file)
    
    if not coordinates:
        print("Error: No valid coordinates found in data file")
        return False
    
    data_length = len(coordinates)
    print(f"Found {data_length} GPS coordinates")
    
    # Get Blender object
    try:
        cansat = bpy.data.objects[object_name]
        cansat.select_set(True)
    except KeyError:
        print(f"Error: Object '{object_name}' not found in scene")
        print("Available objects:", list(bpy.data.objects.keys()))
        return False
    
    # Set animation frame range
    bpy.context.scene.frame_start = 0
    bpy.context.scene.frame_end = data_length - 1
    bpy.context.scene.render.fps = 4  # 4 frames per second
    
    # Set initial position
    cansat.location = (0.0, 0.0, 0.0)
    
    # Create keyframes for each coordinate
    for i in range(data_length):
        if i == 0:
            # First frame: start at origin
            coords_3d = (0.0, 0.0, 0.0)
        else:
            # Calculate relative position from previous coordinate
            previous = coordinates[i - 1]
            current = coordinates[i]
            coords_3d = convert_to_3d_coords(current, previous)
            
            # Accumulate position (relative to previous)
            if i > 1:
                prev_obj = bpy.data.objects[object_name]
                prev_coords_3d = convert_to_3d_coords(coordinates[i - 1], coordinates[i - 2])
                coords_3d = (
                    prev_obj.location.x + coords_3d[0],
                    prev_obj.location.y + coords_3d[1],
                    prev_obj.location.z + coords_3d[2]
                )
        
        # Set object location
        cansat.location = coords_3d
        
        # Insert keyframe
        cansat.keyframe_insert(data_path="location", frame=i)
        
        if i % 10 == 0:  # Print progress every 10 frames
            print(f"Frame {i}/{data_length - 1}: Position {cansat.location}")
    
    print(f"Animation created with {data_length} keyframes")
    return True


def main():
    """Main entry point."""
    # Update this path to your data file location
    # Default assumes data file is in Raket/ directory
    script_dir = Path(__file__).parent
    data_file = script_dir.parent.parent / "data" / "gps_data.txt"
    
    # Alternative: Use absolute path
    # data_file = "C:/Users/JeanK/OneDrive/Bureaublad/Cansat_code/Cansat_AlphaSpot/Raket/data.txt"
    
    if not data_file.exists():
        print(f"Warning: Data file not found at {data_file}")
        print("Please update the data_file path in the script")
        return
    
    print(f"Using data file: {data_file}")
    print(f"Current working directory: {os.getcwd()}")
    
    # Create animation
    success = create_animation(str(data_file))
    
    if success:
        print("Animation created successfully!")
        print("Press Space to play animation in Blender")
    else:
        print("Failed to create animation. Check errors above.")


if __name__ == "__main__":
    main()

