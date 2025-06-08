// PerlinNoise.cs
// Multi-octave Perlin noise implementation for procedural terrain generation.
// Supports fractal Brownian motion (fBm), domain warping, and ridged noise.
//
// Based on Ken Perlin's improved noise function with custom extensions
// for terrain-specific features like erosion simulation and biome mapping.

using System;
using Stride.Core.Mathematics;

namespace ProceduralTerrain.Generation
{
    /// <summary>
    /// GPU-friendly Perlin noise with support for multi-octave fractal noise,
    /// domain warping, and terrain-specific noise variants.
    /// </summary>
    public class PerlinNoise
    {
        private readonly int[] _permutation;
        private readonly int _seed;

        public PerlinNoise(int seed = 0)
        {
            _seed = seed;
            _permutation = GeneratePermutation(seed);
        }

        // ============================================================
        // Core Perlin Noise
        // ============================================================

        /// <summary>
        /// 2D Perlin noise in range [-1, 1].
        /// </summary>
        public float Noise2D(float x, float y)
        {
            int xi = (int)MathF.Floor(x) & 255;
            int yi = (int)MathF.Floor(y) & 255;

            float xf = x - MathF.Floor(x);
            float yf = y - MathF.Floor(y);

            float u = Fade(xf);
            float v = Fade(yf);

            int aa = _permutation[_permutation[xi] + yi];
            int ab = _permutation[_permutation[xi] + yi + 1];
            int ba = _permutation[_permutation[xi + 1] + yi];
            int bb = _permutation[_permutation[xi + 1] + yi + 1];

            float x1 = Lerp(Gradient(aa, xf, yf), Gradient(ba, xf - 1, yf), u);
            float x2 = Lerp(Gradient(ab, xf, yf - 1), Gradient(bb, xf - 1, yf - 1), u);

            return Lerp(x1, x2, v);
        }

        /// <summary>
        /// 3D Perlin noise in range [-1, 1].
        /// </summary>
        public float Noise3D(float x, float y, float z)
        {
            int xi = (int)MathF.Floor(x) & 255;
            int yi = (int)MathF.Floor(y) & 255;
            int zi = (int)MathF.Floor(z) & 255;

            float xf = x - MathF.Floor(x);
            float yf = y - MathF.Floor(y);
            float zf = z - MathF.Floor(z);

            float u = Fade(xf);
            float v = Fade(yf);
            float w = Fade(zf);

            int aaa = _permutation[_permutation[_permutation[xi] + yi] + zi];
            int aba = _permutation[_permutation[_permutation[xi] + yi + 1] + zi];
            int aab = _permutation[_permutation[_permutation[xi] + yi] + zi + 1];
            int abb = _permutation[_permutation[_permutation[xi] + yi + 1] + zi + 1];
            int baa = _permutation[_permutation[_permutation[xi + 1] + yi] + zi];
            int bba = _permutation[_permutation[_permutation[xi + 1] + yi + 1] + zi];
            int bab = _permutation[_permutation[_permutation[xi + 1] + yi] + zi + 1];
            int bbb = _permutation[_permutation[_permutation[xi + 1] + yi + 1] + zi + 1];

            float x1 = Lerp(Gradient3D(aaa, xf, yf, zf), Gradient3D(baa, xf - 1, yf, zf), u);
            float x2 = Lerp(Gradient3D(aba, xf, yf - 1, zf), Gradient3D(bba, xf - 1, yf - 1, zf), u);
            float y1 = Lerp(x1, x2, v);

            x1 = Lerp(Gradient3D(aab, xf, yf, zf - 1), Gradient3D(bab, xf - 1, yf, zf - 1), u);
            x2 = Lerp(Gradient3D(abb, xf, yf - 1, zf - 1), Gradient3D(bbb, xf - 1, yf - 1, zf - 1), u);
            float y2 = Lerp(x1, x2, v);

            return Lerp(y1, y2, w);
        }

        // ============================================================
        // Fractal Brownian Motion (fBm)
        // ============================================================

        /// <summary>
        /// Multi-octave fractal noise for natural-looking terrain.
        /// </summary>
        /// <param name="x">X coordinate</param>
        /// <param name="y">Y coordinate</param>
        /// <param name="octaves">Number of noise layers (4-8 typical)</param>
        /// <param name="lacunarity">Frequency multiplier per octave (usually 2.0)</param>
        /// <param name="persistence">Amplitude decay per octave (usually 0.5)</param>
        /// <returns>Noise value, roughly in [-1, 1]</returns>
        public float FBM(float x, float y, int octaves = 6,
                         float lacunarity = 2.0f, float persistence = 0.5f)
        {
            float total = 0f;
            float amplitude = 1f;
            float frequency = 1f;
            float maxAmplitude = 0f;

            for (int i = 0; i < octaves; i++)
            {
                total += Noise2D(x * frequency, y * frequency) * amplitude;
                maxAmplitude += amplitude;
                amplitude *= persistence;
                frequency *= lacunarity;
            }

            return total / maxAmplitude;
        }

        /// <summary>
        /// Ridged multi-fractal noise — produces sharp mountain ridges.
        /// </summary>
        public float RidgedFBM(float x, float y, int octaves = 6,
                               float lacunarity = 2.0f, float persistence = 0.5f,
                               float ridgeOffset = 1.0f)
        {
            float total = 0f;
            float amplitude = 1f;
            float frequency = 1f;
            float weight = 1f;

            for (int i = 0; i < octaves; i++)
            {
                float noise = MathF.Abs(Noise2D(x * frequency, y * frequency));
                noise = ridgeOffset - noise;
                noise *= noise;
                noise *= weight;

                weight = MathUtil.Clamp(noise * 2f, 0f, 1f);
                total += noise * amplitude;
                amplitude *= persistence;
                frequency *= lacunarity;
            }

            return total;
        }

        // ============================================================
        // Domain Warping
        // ============================================================

        /// <summary>
        /// Warps the input domain for more organic, twisted terrain features.
        /// </summary>
        public float WarpedNoise(float x, float y, float warpStrength = 4.0f,
                                 int octaves = 6)
        {
            float warpX = FBM(x, y, octaves);
            float warpY = FBM(x + 5.2f, y + 1.3f, octaves);

            return FBM(
                x + warpX * warpStrength,
                y + warpY * warpStrength,
                octaves
            );
        }

        // ============================================================
        // Terrain-Specific Helpers
        // ============================================================

        /// <summary>
        /// Generates a height value suitable for terrain with configurable features.
        /// Combines continental shelf, mountain ridges, and detail noise.
        /// </summary>
        public float TerrainHeight(float x, float y, TerrainConfig config)
        {
            // Layer 1: Continental shape (low frequency)
            float continental = FBM(
                x * config.ContinentalScale,
                y * config.ContinentalScale,
                4, 2.0f, 0.5f
            );

            // Layer 2: Mountain ridges
            float ridges = RidgedFBM(
                x * config.MountainScale,
                y * config.MountainScale,
                5, 2.2f, 0.45f
            );

            // Layer 3: Detail noise
            float detail = FBM(
                x * config.DetailScale,
                y * config.DetailScale,
                3, 2.5f, 0.4f
            );

            // Layer 4: Domain warping for erosion-like features
            float warped = WarpedNoise(
                x * config.ErosionScale,
                y * config.ErosionScale,
                config.ErosionStrength,
                3
            );

            // Combine layers
            float height = continental * config.ContinentalWeight
                         + ridges * config.MountainWeight
                         + detail * config.DetailWeight
                         + warped * config.ErosionWeight;

            // Apply global height curve (flattens valleys, sharpens peaks)
            height = MathF.Pow(MathF.Abs(height), config.HeightExponent)
                     * MathF.Sign(height);

            return height * config.MaxHeight;
        }

        // ============================================================
        // Internal Helpers
        // ============================================================

        private static float Fade(float t) => t * t * t * (t * (t * 6 - 15) + 10);

        private static float Lerp(float a, float b, float t) => a + t * (b - a);

        private static float Gradient(int hash, float x, float y)
        {
            return (hash & 3) switch
            {
                0 => x + y,
                1 => -x + y,
                2 => x - y,
                3 => -x - y,
                _ => 0
            };
        }

        private static float Gradient3D(int hash, float x, float y, float z)
        {
            int h = hash & 15;
            float u = h < 8 ? x : y;
            float v = h < 4 ? y : (h == 12 || h == 14 ? x : z);
            return ((h & 1) == 0 ? u : -u) + ((h & 2) == 0 ? v : -v);
        }

        private static int[] GeneratePermutation(int seed)
        {
            var rng = new Random(seed);
            var perm = new int[512];
            var source = new int[256];

            for (int i = 0; i < 256; i++) source[i] = i;

            // Fisher-Yates shuffle
            for (int i = 255; i > 0; i--)
            {
                int j = rng.Next(i + 1);
                (source[i], source[j]) = (source[j], source[i]);
            }

            // Duplicate for wrapping
            for (int i = 0; i < 512; i++)
                perm[i] = source[i & 255];

            return perm;
        }
    }

    /// <summary>
    /// Configuration for multi-layer terrain height generation.
    /// </summary>
    public class TerrainConfig
    {
        public float MaxHeight { get; set; } = 100f;
        public float HeightExponent { get; set; } = 1.2f;

        // Continental (large landmass shapes)
        public float ContinentalScale { get; set; } = 0.002f;
        public float ContinentalWeight { get; set; } = 0.5f;

        // Mountains (ridged fractal)
        public float MountainScale { get; set; } = 0.008f;
        public float MountainWeight { get; set; } = 0.3f;

        // Detail (small features)
        public float DetailScale { get; set; } = 0.05f;
        public float DetailWeight { get; set; } = 0.1f;

        // Erosion (domain warping)
        public float ErosionScale { get; set; } = 0.01f;
        public float ErosionStrength { get; set; } = 3.0f;
        public float ErosionWeight { get; set; } = 0.1f;
    }
}

    // ============================================================
    // Utility
    // ============================================================

    /// <summary>
    /// Sample raw Perlin noise at a 2D position (normalized 0-1 output).
    /// </summary>
    public float Sample2D(float x, float y)
    {
        return (Noise(x * 0.01f, y * 0.01f) + 1f) * 0.5f;
    }

    /// <summary>
    /// Octave noise with configurable lacunarity and persistence.
    /// </summary>
    public float OctaveNoise(float x, float y, int octaves, float lacunarity = 2f, float persistence = 0.5f)
    {
        float value = 0f, amplitude = 1f, frequency = 1f, maxValue = 0f;
        for (int i = 0; i < octaves; i++)
        {
            value += Noise(x * frequency, y * frequency) * amplitude;
            maxValue += amplitude;
            amplitude *= persistence;
            frequency *= lacunarity;
        }
        return value / maxValue;
    }
