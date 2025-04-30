"""
This module defines the Environment class for a 3D scene using VPython.
"""
from vpython import box, sphere, vector, color
import random

class Environment:
    def __init__(self, scene):
        self.scene = scene
        self.ground_tiles = []
        self.buildings = []
        self.hills = []

    def build(self):
        self.ground = box(
            pos=vector(0, -1, 0),
            size=vector(10000, 2, 10000),
            color=vector(0.2,0.6,0.2)
        )

        self.runway = box(
            pos=vector(0, 0, 0),
            size=vector(40, 0.1, 5000),
            color=color.gray(0.5)
        )

        self.sky = sphere(
            pos=vector(0,0,0),
            radius=12000,
            color=vector(0.53,0.81,0.92),
            opacity=0.2
        )

        self.pillar = box(
            pos=vector(0, 5, 0),
            size=vector(5, 10, 5),
            color=color.red
        )

        self._make_ground_tiles(spacing=500, size=5)
        self._make_random_buildings(count=20, range_xz=1500)
        self._make_distant_hills(count=10, range_xz=3000)

    def _make_ground_tiles(self, spacing=500, size=5):
        for x in range(-5000, 5001, spacing):
            for z in range(-5000, 5001, spacing):
                tile = box(
                    pos=vector(x, -0.9, z),
                    size=vector(size, 0.2, size),
                    color=color.white,
                    opacity=0.1
                )
                self.ground_tiles.append(tile)

    def _make_random_buildings(self, count=20, range_xz=1000):
        for _ in range(count):
            x = random.uniform(-range_xz, range_xz)
            z = random.uniform(-range_xz, range_xz)
            height = random.uniform(10, 50)
            building = box(
                pos=vector(x, height/2, z),
                size=vector(random.uniform(10,20), height, random.uniform(10,20)),
                color=color.gray(0.7)
            )
            self.buildings.append(building)

    def _make_distant_hills(self, count=10, range_xz=3000):
        for _ in range(count):
            x = random.uniform(-range_xz, range_xz)
            z = random.uniform(-range_xz, range_xz)
            hill = sphere(
                pos=vector(x, -0.5, z),
                radius=random.uniform(300, 600),
                color=vector(0.3,0.5,0.3),
                opacity=0.3
            )
            self.hills.append(hill)
