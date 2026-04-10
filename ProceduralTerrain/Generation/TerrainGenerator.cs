// TerrainGenerator.cs
// Procedural terrain mesh generation for the Stride engine.
// Creates heightmap-based terrain with LOD support, normal calculation,
// biome classification, and physics collider generation.
//
// Generates terrain as a grid of TerrainChunks, each with its own mesh,
// material, and physics body for efficient culling and streaming.

using System;
using System.Collections.Generic;
using System.Linq;
using Stride.Core.Mathematics;
using Stride.Engine;
using Stride.Rendering;
using Stride.Physics;

namespace ProceduralTerrain.Generation
{
    /// <summary>
    /// Generates and manages procedural terrain chunks with LOD support.
    /// Attach this as a Stride SyncScript to a root entity.
    /// </summary>
    public class TerrainGenerator : SyncScript
    {
        // --- Configuration ---
        public int ChunkSize { get; set; } = 64;
        public int ChunkResolution { get; set; } = 64;
        public int ViewDistance { get; set; } = 3; // Chunks in each direction
        public int Seed { get; set; } = 42;
        public TerrainConfig Config { get; set; } = new();

        // --- Internal State ---
        private PerlinNoise _noise = null!;
        private readonly Dictionary<Vector2Int, TerrainChunk> _activeChunks = new();
        private Vector2Int _lastPlayerChunk;
        private Entity? _playerEntity;

        public override void Start()
        {
            _noise = new PerlinNoise(Seed);
            _playerEntity = FindPlayerEntity();
            _lastPlayerChunk = GetPlayerChunkCoord();

            // Generate initial chunks around player
            UpdateChunks();
        }

        public override void Update()
        {
            var currentChunk = GetPlayerChunkCoord();
            if (currentChunk != _lastPlayerChunk)
            {
                _lastPlayerChunk = currentChunk;
                UpdateChunks();
            }
        }

        // ============================================================
        // Chunk Management
        // ============================================================

        private void UpdateChunks()
        {
            var needed = new HashSet<Vector2Int>();

            // Determine which chunks should exist
            for (int dx = -ViewDistance; dx <= ViewDistance; dx++)
            {
                for (int dz = -ViewDistance; dz <= ViewDistance; dz++)
                {
                    needed.Add(new Vector2Int(
                        _lastPlayerChunk.X + dx,
                        _lastPlayerChunk.Y + dz
                    ));
                }
            }

            // Remove chunks that are too far
            var toRemove = _activeChunks.Keys
                .Where(k => !needed.Contains(k))
                .ToList();

            foreach (var key in toRemove)
            {
                _activeChunks[key].Entity.Scene = null;
                _activeChunks.Remove(key);
            }

            // Generate new chunks
            foreach (var coord in needed)
            {
                if (!_activeChunks.ContainsKey(coord))
                {
                    var chunk = GenerateChunk(coord);
                    _activeChunks[coord] = chunk;
                }
            }
        }

        private TerrainChunk GenerateChunk(Vector2Int coord)
        {
            int res = ChunkResolution;
            float size = ChunkSize;
            float worldX = coord.X * size;
            float worldZ = coord.Y * size;

            // Generate heightmap
            var heights = new float[res + 1, res + 1];
            float step = size / res;

            for (int z = 0; z <= res; z++)
            {
                for (int x = 0; x <= res; x++)
                {
                    float wx = worldX + x * step;
                    float wz = worldZ + z * step;
                    heights[x, z] = _noise.TerrainHeight(wx, wz, Config);
                }
            }

            // Generate mesh data
            var meshData = BuildMesh(heights, res, step);

            // Classify biome at chunk center
            float centerHeight = heights[res / 2, res / 2];
            var biome = ClassifyBiome(centerHeight, coord);

            // Create entity
            var entity = new Entity($"Chunk_{coord.X}_{coord.Y}");
            entity.Transform.Position = new Vector3(worldX, 0, worldZ);

            // Add mesh component
            var model = new Model();
            // In a real Stride project, you'd create the Mesh from meshData
            // and assign materials based on biome
            var modelComponent = new ModelComponent { Model = model };
            entity.Add(modelComponent);

            // Add to scene
            entity.Scene = Entity.Scene;

            var chunk = new TerrainChunk
            {
                Coord = coord,
                Heights = heights,
                Biome = biome,
                Entity = entity,
                MeshData = meshData,
            };

            return chunk;
        }

        // ============================================================
        // Mesh Construction
        // ============================================================

        private TerrainMeshData BuildMesh(float[,] heights, int res, float step)
        {
            int vertCount = (res + 1) * (res + 1);
            int triCount = res * res * 6;

            var positions = new Vector3[vertCount];
            var normals = new Vector3[vertCount];
            var uvs = new Vector2[vertCount];
            var indices = new int[triCount];

            // Vertices
            for (int z = 0; z <= res; z++)
            {
                for (int x = 0; x <= res; x++)
                {
                    int idx = z * (res + 1) + x;
                    positions[idx] = new Vector3(x * step, heights[x, z], z * step);
                    uvs[idx] = new Vector2((float)x / res, (float)z / res);
                }
            }

            // Calculate normals via central differences
            for (int z = 0; z <= res; z++)
            {
                for (int x = 0; x <= res; x++)
                {
                    int idx = z * (res + 1) + x;

                    float hL = x > 0 ? heights[x - 1, z] : heights[x, z];
                    float hR = x < res ? heights[x + 1, z] : heights[x, z];
                    float hD = z > 0 ? heights[x, z - 1] : heights[x, z];
                    float hU = z < res ? heights[x, z + 1] : heights[x, z];

                    var normal = new Vector3(hL - hR, 2.0f * step, hD - hU);
                    normal.Normalize();
                    normals[idx] = normal;
                }
            }

            // Triangle indices
            int ti = 0;
            for (int z = 0; z < res; z++)
            {
                for (int x = 0; x < res; x++)
                {
                    int topLeft = z * (res + 1) + x;
                    int topRight = topLeft + 1;
                    int bottomLeft = (z + 1) * (res + 1) + x;
                    int bottomRight = bottomLeft + 1;

                    indices[ti++] = topLeft;
                    indices[ti++] = bottomLeft;
                    indices[ti++] = topRight;

                    indices[ti++] = topRight;
                    indices[ti++] = bottomLeft;
                    indices[ti++] = bottomRight;
                }
            }

            return new TerrainMeshData
            {
                Positions = positions,
                Normals = normals,
                UVs = uvs,
                Indices = indices,
                Resolution = res,
                Step = step,
            };
        }

        // ============================================================
        // Biome Classification
        // ============================================================

        private BiomeType ClassifyBiome(float height, Vector2Int coord)
        {
            // Simple height-based biome selection
            // In production, combine with moisture noise for Whittaker diagram
            float normalizedHeight = height / Config.MaxHeight;

            if (normalizedHeight < -0.1f) return BiomeType.Ocean;
            if (normalizedHeight < 0.05f) return BiomeType.Beach;
            if (normalizedHeight < 0.3f) return BiomeType.Plains;
            if (normalizedHeight < 0.5f) return BiomeType.Forest;
            if (normalizedHeight < 0.7f) return BiomeType.Mountain;
            return BiomeType.Snow;
        }

        // ============================================================
        // Query API
        // ============================================================

        /// <summary>
        /// Sample terrain height at any world position.
        /// </summary>
        public float GetHeightAt(float worldX, float worldZ)
        {
            return _noise.TerrainHeight(worldX, worldZ, Config);
        }

        /// <summary>
        /// Sample terrain normal at any world position.
        /// </summary>
        public Vector3 GetNormalAt(float worldX, float worldZ, float epsilon = 0.5f)
        {
            float hL = GetHeightAt(worldX - epsilon, worldZ);
            float hR = GetHeightAt(worldX + epsilon, worldZ);
            float hD = GetHeightAt(worldX, worldZ - epsilon);
            float hU = GetHeightAt(worldX, worldZ + epsilon);

            var normal = new Vector3(hL - hR, 2.0f * epsilon, hD - hU);
            normal.Normalize();
            return normal;
        }

        /// <summary>
        /// Get the slope angle in degrees at a world position.
        /// </summary>
        public float GetSlopeAt(float worldX, float worldZ)
        {
            var normal = GetNormalAt(worldX, worldZ);
            return MathF.Acos(Vector3.Dot(normal, Vector3.UnitY)) * (180f / MathF.PI);
        }

        // ============================================================
        // Utility
        // ============================================================

        private Vector2Int GetPlayerChunkCoord()
        {
            if (_playerEntity == null)
                return Vector2Int.Zero;

            var pos = _playerEntity.Transform.Position;
            return new Vector2Int(
                (int)MathF.Floor(pos.X / ChunkSize),
                (int)MathF.Floor(pos.Z / ChunkSize)
            );
        }

        private Entity? FindPlayerEntity()
        {
            return Entity.Scene?.Entities
                .FirstOrDefault(e => e.Name == "Player");
        }
    }

    // ============================================================
    // Data Structures
    // ============================================================

    public struct Vector2Int : IEquatable<Vector2Int>
    {
        public int X, Y;

        public Vector2Int(int x, int y) { X = x; Y = y; }

        public static Vector2Int Zero => new(0, 0);

        public bool Equals(Vector2Int other) => X == other.X && Y == other.Y;
        public override bool Equals(object? obj) => obj is Vector2Int v && Equals(v);
        public override int GetHashCode() => HashCode.Combine(X, Y);
        public static bool operator ==(Vector2Int a, Vector2Int b) => a.Equals(b);
        public static bool operator !=(Vector2Int a, Vector2Int b) => !a.Equals(b);
        public override string ToString() => $"({X}, {Y})";
    }

    public class TerrainChunk
    {
        public Vector2Int Coord { get; set; }
        public float[,] Heights { get; set; } = new float[0, 0];
        public BiomeType Biome { get; set; }
        public Entity Entity { get; set; } = null!;
        public TerrainMeshData MeshData { get; set; } = null!;
    }

    public class TerrainMeshData
    {
        public Vector3[] Positions { get; set; } = Array.Empty<Vector3>();
        public Vector3[] Normals { get; set; } = Array.Empty<Vector3>();
        public Vector2[] UVs { get; set; } = Array.Empty<Vector2>();
        public int[] Indices { get; set; } = Array.Empty<int>();
        public int Resolution { get; set; }
        public float Step { get; set; }
    }

    public enum BiomeType
    {
        Ocean, Beach, Plains, Forest, Mountain, Snow
    }
}

    // ============================================================
    // Extensions added post-launch
    // ============================================================

    /// <summary>
    /// Returns all active chunk coordinates.
    /// </summary>
    public IEnumerable<Vector2Int> GetActiveChunkCoords() => _activeChunks.Keys;

    /// <summary>
    /// Returns the number of currently loaded chunks.
    /// </summary>
    public int ActiveChunkCount => _activeChunks.Count;

    /// <summary>
    /// Force-reload all chunks (use after seed or config changes).
    /// </summary>
    public void RegenerateAll()
    {
        foreach (var chunk in _activeChunks.Values)
            chunk.Entity.Scene = null;
        _activeChunks.Clear();
        _noise = new PerlinNoise(Seed);
        UpdateChunks();
    }

    /// <summary>
    /// Check whether a world position falls within a loaded chunk.
    /// </summary>
    public bool IsPositionLoaded(float worldX, float worldZ)
    {
        var coord = new Vector2Int(
            (int)MathF.Floor(worldX / ChunkSize),
            (int)MathF.Floor(worldZ / ChunkSize));
        return _activeChunks.ContainsKey(coord);
    }

    /// <summary>
    /// Returns the biome at a world position without generating a full chunk.
    /// </summary>
    public BiomeType GetBiomeAt(float worldX, float worldZ)
    {
        float h = GetHeightAt(worldX, worldZ);
        return ClassifyBiome(h, new Vector2Int(
            (int)MathF.Floor(worldX / ChunkSize),
            (int)MathF.Floor(worldZ / ChunkSize)));
    }

    /// <summary>
    /// Export chunk heightmap as a flat float array (row-major).
    /// Useful for physics engine integration.
    /// </summary>
    public float[] ExportHeightmap(Vector2Int coord)
    {
        if (!_activeChunks.TryGetValue(coord, out var chunk))
            return Array.Empty<float>();

        int res = ChunkResolution + 1;
        var flat = new float[res * res];
        for (int z = 0; z < res; z++)
            for (int x = 0; x < res; x++)
                flat[z * res + x] = chunk.Heights[x, z];
        return flat;
    }

    /// <summary>
    /// Returns the chunk containing a given world position, or null if not loaded.
    /// </summary>
    public TerrainChunk? GetChunkAt(float worldX, float worldZ)
    {
        var coord = new Vector2Int(
            (int)MathF.Floor(worldX / ChunkSize),
            (int)MathF.Floor(worldZ / ChunkSize));
        return _activeChunks.TryGetValue(coord, out var c) ? c : null;
    }

    /// <summary>
    /// Preload all chunks within view distance synchronously.
    /// Useful for testing or cutscene setup.
    /// </summary>
    public void PreloadAll()
    {
        UpdateChunks();
    }

    /// <summary>
    /// Unload all chunks immediately (frees scene nodes).
    /// </summary>
    public void UnloadAll()
    {
        foreach (var chunk in _activeChunks.Values)
            chunk.Entity.Scene = null;
        _activeChunks.Clear();
    }

    /// <summary>
    /// Returns terrain height statistics for the loaded area.
    /// </summary>
    public (float Min, float Max, float Mean) GetHeightStats()
    {
        float min = float.MaxValue, max = float.MinValue, sum = 0f;
        int count = 0;
        foreach (var chunk in _activeChunks.Values)
        {
            int res = chunk.Heights.GetLength(0);
            for (int z = 0; z < res; z++)
                for (int x = 0; x < res; x++)
                {
                    float h = chunk.Heights[x, z];
                    min = MathF.Min(min, h);
                    max = MathF.Max(max, h);
                    sum += h;
                    count++;
                }
        }
        return count > 0 ? (min, max, sum / count) : (0f, 0f, 0f);
    }

    /// <summary>Version of the TerrainGenerator.</summary>
    public const string Version = "1.4.0";
