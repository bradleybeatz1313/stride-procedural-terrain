// NavigationAgent.cs
// ML-enhanced navigation agent that uses terrain awareness and learned
// heuristics to navigate procedural terrain efficiently.
//
// Combines traditional A* pathfinding with a neural cost estimator
// that learns terrain traversal costs from agent experience.
// The cost network predicts movement cost based on slope, biome,
// and terrain features — enabling agents to learn that snow is
// slippery, steep slopes are slow, and forests provide cover.

using System;
using System.Collections.Generic;
using System.Linq;
using Stride.Core.Mathematics;
using Stride.Engine;

namespace ProceduralTerrain.Navigation
{
    /// <summary>
    /// AI navigation agent with ML-augmented pathfinding.
    /// Learns optimal terrain traversal from experience.
    /// </summary>
    public class NavigationAgent : SyncScript
    {
        // --- Configuration ---
        public float MoveSpeed { get; set; } = 8.0f;
        public float TurnSpeed { get; set; } = 180.0f; // Degrees/sec
        public float WaypointReachDistance { get; set; } = 2.0f;
        public float PathRecalcInterval { get; set; } = 2.0f;
        public int MaxPathNodes { get; set; } = 200;
        public float GridCellSize { get; set; } = 2.0f;

        // --- State ---
        private TerrainGenerator? _terrain;
        private CostNetwork _costNetwork = null!;
        private List<Vector3> _currentPath = new();
        private int _pathIndex;
        private Vector3 _targetPosition;
        private bool _hasTarget;
        private float _pathTimer;
        private readonly List<TraversalSample> _experienceBuffer = new();

        // --- Telemetry ---
        public int PathsComputed { get; private set; }
        public float TotalDistanceTraveled { get; private set; }
        public int WaypointsReached { get; private set; }

        public override void Start()
        {
            _terrain = Entity.Scene?.Entities
                .SelectMany(e => e.GetAll<TerrainGenerator>())
                .FirstOrDefault();

            _costNetwork = new CostNetwork(inputDim: 6, hiddenDim: 16);
        }

        public override void Update()
        {
            if (!_hasTarget) return;

            float dt = (float)Game.UpdateTime.Elapsed.TotalSeconds;
            _pathTimer -= dt;

            // Recalculate path periodically
            if (_pathTimer <= 0 || _currentPath.Count == 0)
            {
                ComputePath(_targetPosition);
                _pathTimer = PathRecalcInterval;
            }

            // Follow path
            if (_pathIndex < _currentPath.Count)
            {
                FollowPath(dt);
            }
        }

        // ============================================================
        // Public API
        // ============================================================

        /// <summary>
        /// Set a navigation target. Agent will pathfind and move toward it.
        /// </summary>
        public void NavigateTo(Vector3 target)
        {
            _targetPosition = target;
            _hasTarget = true;
            ComputePath(target);
        }

        /// <summary>
        /// Stop navigation and clear the current path.
        /// </summary>
        public void Stop()
        {
            _hasTarget = false;
            _currentPath.Clear();
        }

        /// <summary>
        /// Train the cost network on accumulated experience.
        /// Call periodically (e.g., every 100 traversals).
        /// </summary>
        public void TrainFromExperience(int epochs = 10)
        {
            if (_experienceBuffer.Count < 10) return;
            _costNetwork.Train(_experienceBuffer, epochs);
            _experienceBuffer.Clear();
        }

        // ============================================================
        // Pathfinding (A* with ML Cost Estimation)
        // ============================================================

        private void ComputePath(Vector3 target)
        {
            if (_terrain == null) return;

            var startGrid = WorldToGrid(Entity.Transform.Position);
            var endGrid = WorldToGrid(target);

            var path = AStarSearch(startGrid, endGrid);

            _currentPath = path.Select(GridToWorld).ToList();
            _pathIndex = 0;
            PathsComputed++;
        }

        private List<Vector2Int> AStarSearch(Vector2Int start, Vector2Int end)
        {
            var openSet = new PriorityQueue<Vector2Int, float>();
            var cameFrom = new Dictionary<Vector2Int, Vector2Int>();
            var gScore = new Dictionary<Vector2Int, float> { [start] = 0 };
            var fScore = new Dictionary<Vector2Int, float>();

            fScore[start] = Heuristic(start, end);
            openSet.Enqueue(start, fScore[start]);

            int iterations = 0;

            while (openSet.Count > 0 && iterations < MaxPathNodes * 10)
            {
                iterations++;
                var current = openSet.Dequeue();

                if (current == end)
                    return ReconstructPath(cameFrom, current);

                foreach (var neighbor in GetNeighbors(current))
                {
                    float moveCost = EstimateMoveCost(current, neighbor);
                    float tentativeG = gScore.GetValueOrDefault(current, float.MaxValue) + moveCost;

                    if (tentativeG < gScore.GetValueOrDefault(neighbor, float.MaxValue))
                    {
                        cameFrom[neighbor] = current;
                        gScore[neighbor] = tentativeG;
                        fScore[neighbor] = tentativeG + Heuristic(neighbor, end);
                        openSet.Enqueue(neighbor, fScore[neighbor]);
                    }
                }
            }

            // No path found — return direct line
            return new List<Vector2Int> { start, end };
        }

        private float EstimateMoveCost(Vector2Int from, Vector2Int to)
        {
            if (_terrain == null) return 1.0f;

            var fromWorld = GridToWorld(from);
            var toWorld = GridToWorld(to);

            float fromHeight = _terrain.GetHeightAt(fromWorld.X, fromWorld.Z);
            float toHeight = _terrain.GetHeightAt(toWorld.X, toWorld.Z);
            float slope = _terrain.GetSlopeAt(toWorld.X, toWorld.Z);

            // Build feature vector for cost network
            float[] features = new float[]
            {
                (toHeight - fromHeight) / 10f,  // Height difference
                slope / 90f,                     // Normalized slope
                fromHeight / 100f,               // Absolute height
                Vector2.Distance(
                    new Vector2(from.X, from.Y),
                    new Vector2(to.X, to.Y)
                ),                               // Grid distance
                MathF.Abs(toHeight),             // Height magnitude
                slope > 45 ? 1f : 0f,           // Steep flag
            };

            // ML-predicted cost (learned from experience)
            float mlCost = _costNetwork.Predict(features);

            // Base cost from slope physics
            float slopeCost = 1.0f + MathF.Pow(slope / 45f, 2) * 3.0f;

            // Impassable if too steep
            if (slope > 60f) return float.MaxValue;

            // Blend ML prediction with physics-based cost
            float blendFactor = MathF.Min(_costNetwork.TrainingSteps / 1000f, 0.7f);
            return MathUtil.Lerp(slopeCost, mlCost, blendFactor);
        }

        private float Heuristic(Vector2Int a, Vector2Int b)
        {
            // Euclidean distance with height penalty
            float dx = a.X - b.X;
            float dy = a.Y - b.Y;
            float dist = MathF.Sqrt(dx * dx + dy * dy) * GridCellSize;

            if (_terrain != null)
            {
                var aWorld = GridToWorld(a);
                var bWorld = GridToWorld(b);
                float heightDiff = MathF.Abs(
                    _terrain.GetHeightAt(aWorld.X, aWorld.Z) -
                    _terrain.GetHeightAt(bWorld.X, bWorld.Z)
                );
                dist += heightDiff * 0.5f;
            }

            return dist;
        }

        // ============================================================
        // Movement
        // ============================================================

        private void FollowPath(float dt)
        {
            var targetWP = _currentPath[_pathIndex];
            var pos = Entity.Transform.Position;
            var direction = targetWP - pos;
            direction.Y = 0; // Horizontal movement only

            float distance = direction.Length();

            if (distance < WaypointReachDistance)
            {
                // Record traversal experience
                RecordTraversal(pos, targetWP);

                _pathIndex++;
                WaypointsReached++;

                if (_pathIndex >= _currentPath.Count)
                {
                    _hasTarget = false;
                    return;
                }
                return;
            }

            direction.Normalize();

            // Apply terrain-aware speed
            float slope = _terrain?.GetSlopeAt(pos.X, pos.Z) ?? 0f;
            float speedMod = 1.0f - MathUtil.Clamp(slope / 60f, 0f, 0.8f);
            float speed = MoveSpeed * speedMod;

            // Move
            var movement = direction * speed * dt;
            var newPos = pos + movement;

            // Snap to terrain height
            if (_terrain != null)
            {
                newPos.Y = _terrain.GetHeightAt(newPos.X, newPos.Z) + 1.0f;
            }

            TotalDistanceTraveled += movement.Length();
            Entity.Transform.Position = newPos;

            // Face movement direction
            if (direction.LengthSquared() > 0.001f)
            {
                float targetYaw = MathF.Atan2(direction.X, direction.Z) * (180f / MathF.PI);
                var rot = Entity.Transform.RotationEulerXYZ;
                rot.Y = MathUtil.Lerp(rot.Y, targetYaw, TurnSpeed * dt / 180f);
                Entity.Transform.RotationEulerXYZ = rot;
            }
        }

        // ============================================================
        // Experience Recording
        // ============================================================

        private void RecordTraversal(Vector3 from, Vector3 to)
        {
            if (_terrain == null) return;

            float slope = _terrain.GetSlopeAt(to.X, to.Z);
            float heightDiff = to.Y - from.Y;
            float actualCost = Vector3.Distance(from, to);

            // Penalize steep climbs, reward downhill
            if (heightDiff > 0) actualCost *= 1.0f + heightDiff * 0.1f;
            else actualCost *= 1.0f + MathF.Abs(heightDiff) * 0.02f;

            _experienceBuffer.Add(new TraversalSample
            {
                Features = new float[]
                {
                    heightDiff / 10f,
                    slope / 90f,
                    from.Y / 100f,
                    Vector3.Distance(from, to),
                    MathF.Abs(to.Y),
                    slope > 45 ? 1f : 0f,
                },
                ActualCost = actualCost,
            });
        }

        // ============================================================
        // Utility
        // ============================================================

        private static IEnumerable<Vector2Int> GetNeighbors(Vector2Int pos)
        {
            yield return new Vector2Int(pos.X + 1, pos.Y);
            yield return new Vector2Int(pos.X - 1, pos.Y);
            yield return new Vector2Int(pos.X, pos.Y + 1);
            yield return new Vector2Int(pos.X, pos.Y - 1);
            yield return new Vector2Int(pos.X + 1, pos.Y + 1);
            yield return new Vector2Int(pos.X - 1, pos.Y + 1);
            yield return new Vector2Int(pos.X + 1, pos.Y - 1);
            yield return new Vector2Int(pos.X - 1, pos.Y - 1);
        }

        private Vector2Int WorldToGrid(Vector3 world)
        {
            return new Vector2Int(
                (int)MathF.Round(world.X / GridCellSize),
                (int)MathF.Round(world.Z / GridCellSize)
            );
        }

        private Vector3 GridToWorld(Vector2Int grid)
        {
            float x = grid.X * GridCellSize;
            float z = grid.Y * GridCellSize;
            float y = _terrain?.GetHeightAt(x, z) ?? 0;
            return new Vector3(x, y + 1.0f, z);
        }

        private static List<Vector2Int> ReconstructPath(
            Dictionary<Vector2Int, Vector2Int> cameFrom, Vector2Int current)
        {
            var path = new List<Vector2Int> { current };
            while (cameFrom.ContainsKey(current))
            {
                current = cameFrom[current];
                path.Insert(0, current);
            }
            return path;
        }
    }

    // ============================================================
    // Simple Cost Network (single hidden layer)
    // ============================================================

    /// <summary>
    /// Lightweight neural network for learning terrain traversal costs.
    /// Single hidden layer with ReLU activation. Trained via simple SGD.
    /// </summary>
    public class CostNetwork
    {
        private readonly float[,] _weightsIH;
        private readonly float[] _biasH;
        private readonly float[] _weightsHO;
        private float _biasO;
        private readonly int _inputDim;
        private readonly int _hiddenDim;
        private readonly Random _rng;

        public int TrainingSteps { get; private set; }

        public CostNetwork(int inputDim = 6, int hiddenDim = 16, int seed = 42)
        {
            _inputDim = inputDim;
            _hiddenDim = hiddenDim;
            _rng = new Random(seed);

            _weightsIH = new float[inputDim, hiddenDim];
            _biasH = new float[hiddenDim];
            _weightsHO = new float[hiddenDim];
            _biasO = 0;

            // Xavier initialization
            float limit = MathF.Sqrt(6.0f / (inputDim + hiddenDim));
            for (int i = 0; i < inputDim; i++)
                for (int j = 0; j < hiddenDim; j++)
                    _weightsIH[i, j] = ((float)_rng.NextDouble() * 2 - 1) * limit;

            limit = MathF.Sqrt(6.0f / (hiddenDim + 1));
            for (int j = 0; j < hiddenDim; j++)
                _weightsHO[j] = ((float)_rng.NextDouble() * 2 - 1) * limit;
        }

        public float Predict(float[] input)
        {
            var hidden = new float[_hiddenDim];

            // Input → Hidden (ReLU)
            for (int j = 0; j < _hiddenDim; j++)
            {
                float sum = _biasH[j];
                for (int i = 0; i < _inputDim; i++)
                    sum += input[i] * _weightsIH[i, j];
                hidden[j] = MathF.Max(0, sum); // ReLU
            }

            // Hidden → Output
            float output = _biasO;
            for (int j = 0; j < _hiddenDim; j++)
                output += hidden[j] * _weightsHO[j];

            return MathF.Max(0.1f, output); // Cost must be positive
        }

        public void Train(List<TraversalSample> samples, int epochs = 10,
                          float learningRate = 0.001f)
        {
            for (int epoch = 0; epoch < epochs; epoch++)
            {
                // Shuffle
                var shuffled = samples.OrderBy(_ => _rng.Next()).ToList();

                foreach (var sample in shuffled)
                {
                    // Forward pass
                    var hidden = new float[_hiddenDim];
                    for (int j = 0; j < _hiddenDim; j++)
                    {
                        float sum = _biasH[j];
                        for (int i = 0; i < _inputDim; i++)
                            sum += sample.Features[i] * _weightsIH[i, j];
                        hidden[j] = MathF.Max(0, sum);
                    }

                    float prediction = _biasO;
                    for (int j = 0; j < _hiddenDim; j++)
                        prediction += hidden[j] * _weightsHO[j];
                    prediction = MathF.Max(0.1f, prediction);

                    // Loss gradient (MSE)
                    float error = prediction - sample.ActualCost;

                    // Backward pass
                    _biasO -= learningRate * error;
                    for (int j = 0; j < _hiddenDim; j++)
                    {
                        float dHO = error * hidden[j];
                        _weightsHO[j] -= learningRate * dHO;

                        if (hidden[j] > 0) // ReLU derivative
                        {
                            float dH = error * _weightsHO[j];
                            _biasH[j] -= learningRate * dH;

                            for (int i = 0; i < _inputDim; i++)
                                _weightsIH[i, j] -= learningRate * dH * sample.Features[i];
                        }
                    }

                    TrainingSteps++;
                }
            }
        }
    }

    public class TraversalSample
    {
        public float[] Features { get; set; } = Array.Empty<float>();
        public float ActualCost { get; set; }
    }
}

    // ─── Debugging Helpers ──────────────────────────────────────────

    /// <summary>
    /// Returns the current path as a read-only list of world positions.
    /// </summary>
    public IReadOnlyList<Vector3> CurrentPath => _currentPath ?? (IReadOnlyList<Vector3>)Array.Empty<Vector3>();

    /// <summary>
    /// Returns how far along the current path the agent is (0-1).
    /// </summary>
    public float PathProgress => _currentPath is { Count: > 0 }
        ? (float)_pathIndex / _currentPath.Count
        : 0f;
