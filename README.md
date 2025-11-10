# 🏔️ Procedural Terrain + ML Navigation — Stride Engine

Procedural terrain generation using multi-octave Perlin noise with an ML-augmented A* navigation agent that learns terrain traversal costs from experience. Built in C# for the Stride engine (formerly Xenko).

![Stride](https://img.shields.io/badge/Stride-4.2-blue)
![C#](https://img.shields.io/badge/C%23-12-purple?logo=dotnet)
![.NET](https://img.shields.io/badge/.NET-8.0-blueviolet)
![License](https://img.shields.io/badge/license-MIT-green)

---

## 🎯 Features

### Procedural Terrain Generation (`Generation/`)
- **Multi-octave Perlin noise** — Full implementation with permutation tables and gradient vectors
- **Fractal Brownian motion** — Configurable octaves, lacunarity, and persistence
- **Ridged multi-fractal** — Sharp mountain ridge generation
- **Domain warping** — Organic, erosion-like terrain distortion
- **Layered terrain height** — Continental shelf + mountain ridges + detail noise + erosion effects
- **Biome classification** — Height-based biome assignment (ocean, beach, plains, forest, mountain, snow)
- **Chunk streaming** — Dynamic chunk loading/unloading based on player proximity
- **Runtime mesh generation** — Vertex positions, normals (central differences), UVs, and index buffers

### ML Navigation Agent (`Navigation/`)
- **Hybrid A* pathfinding** — Traditional A* with ML-predicted edge costs
- **Cost network** — Single-layer neural network learns traversal cost from experience
- **Experience buffer** — Records actual traversal cost vs predicted for online training
- **Terrain-aware speed** — Movement speed scales with slope angle
- **Adaptive blending** — Cost prediction shifts from physics-based to ML-based as training progresses
- **Height snapping** — Agent position automatically projected to terrain surface
- **Telemetry** — Paths computed, distance traveled, waypoints reached

### Noise Library (`Generation/PerlinNoise.cs`)
- **2D and 3D Perlin noise** — Core implementation with smooth interpolation
- **Xavier-seeded permutation** — Deterministic noise from integer seeds
- **TerrainConfig** — Data-driven height generation with 4 configurable noise layers
- **Query API** — `GetHeightAt()`, `GetNormalAt()`, `GetSlopeAt()` for any world position

---

## 📂 Project Structure

```
stride-procedural-terrain/
├── ProceduralTerrain.csproj
├── ProceduralTerrain/
│   ├── Generation/
│   │   ├── PerlinNoise.cs        # Noise library (fBm, ridged, warped)
│   │   └── TerrainGenerator.cs   # Chunk-based terrain mesh generation
│   ├── Navigation/
│   │   └── NavigationAgent.cs    # ML-augmented A* pathfinding
│   ├── Core/
│   └── Utils/
├── Assets/
├── Resources/
└── README.md
```

---

## 🚀 Getting Started

### Requirements
- [Stride Engine 4.2+](https://stride3d.net/download/)
- .NET 8.0 SDK
- Visual Studio 2022 or JetBrains Rider

### Setup
1. Clone the repository
2. Open `ProceduralTerrain.csproj` in Stride Game Studio
3. Add `TerrainGenerator` script to a root entity
4. Add `NavigationAgent` script to an agent entity
5. Build and run

### Configuration
Terrain parameters are fully data-driven via `TerrainConfig`:
```csharp
var config = new TerrainConfig
{
    MaxHeight = 100f,
    ContinentalScale = 0.002f,
    MountainScale = 0.008f,
    DetailScale = 0.05f,
    ErosionStrength = 3.0f,
};
```

---

## 🧪 Architecture Decisions

| Decision | Rationale |
|----------|-----------|
| Stride over Unity/Godot | Open-source C# engine; full source access for ML integration research |
| Chunk-based streaming | Enables infinite terrain without memory limits; natural LOD boundaries |
| ML cost blending | Agent starts with physics-based costs (reliable) and gradually trusts learned costs |
| Online training | Cost network trains from agent's own experience — no external training pipeline needed |
| Central-difference normals | More accurate than face normals for smooth terrain shading |

---

## 🔬 AI Research Applications

- **Learned navigation heuristics** — Cost network demonstrates online ML integration in game pathfinding
- **Terrain complexity control** — Tunable `TerrainConfig` parameters for difficulty scaling
- **Biome-aware navigation** — Extend cost features with biome type for terrain-preference learning
- **Multi-agent coordination** — Multiple `NavigationAgent` instances can share experience buffers
- **Procedural evaluation** — Seed-based deterministic terrain for reproducible agent benchmarks

---

## 📄 License

MIT

---

## Quick Start

1. Open the solution in Visual Studio or Rider (.NET 6 SDK required)
2. Reference the Stride engine NuGet packages (see csproj)
3. Attach `TerrainGenerator` as a SyncScript to a root entity
4. Set `Seed`, `ChunkSize`, and `ViewDistance` in the editor
5. Run -- terrain streams in as the player entity moves

---

## Terrain Configuration

| Property | Default | Description |
|----------|---------|-------------|
| ChunkSize | 64 | World units per chunk |
| ChunkResolution | 64 | Vertices per chunk edge |
| ViewDistance | 3 | Chunks loaded in each direction |
| Seed | 42 | RNG seed for deterministic generation |
| Config.MaxHeight | 80f | Peak terrain height |
| Config.Octaves | 6 | Noise octave count |
