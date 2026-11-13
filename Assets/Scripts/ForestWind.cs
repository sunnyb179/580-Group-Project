 using System;
using UnityEngine;

[DisallowMultipleComponent]
public sealed class ForestWind : MonoBehaviour
{
    private const int GridSize = 64;
    private const float CellWidth = 2f;
    private const int PressureIterations = 40;

    [Tooltip("Assign one root pivot per tree. Leave empty to sway this GameObject.")]
    [SerializeField] private Transform[] treePivots = Array.Empty<Transform>();
    [Tooltip("Offset from this object to the lower X/Z corner of the wind field, in world metres.")]
    [SerializeField] private Vector2 fieldOffset = new Vector2(-44f, -64f);
    [SerializeField] private Vector2 backgroundWind = new Vector2(8f, 0f);
    [SerializeField] private Vector2 gustCenter = new Vector2(28f, 64f);
    [SerializeField, Min(0f)] private float gustAcceleration = 12f;
    [SerializeField, Min(0.1f)] private float gustWidth = 6f;
    [SerializeField, Min(0.1f)] private float gustDuration = 1f;

    [Header("Tree response")]
    [Tooltip("Effective drag torque per unit inertia and squared relative wind speed.")]
    [SerializeField, Min(0f)] private float dragResponse = 0.035f;
    [SerializeField, Range(0.2f, 3f)] private float naturalFrequencyHz = 0.65f;
    [SerializeField, Range(0.05f, 1f)] private float dampingRatio = 0.4f;
    [SerializeField, Min(0f)] private float loadHeight = 3f;
    [SerializeField, Range(1f, 35f)] private float maxLeanDegrees = 20f;

    private sealed class TreeState
    {
        public Transform Pivot;
        public Quaternion RestRotation;
        public Vector2 Lean;
        public Vector2 LeanRate;
    }

    private TreeState[] trees;
    private WindGrid grid;
    private Vector2 fieldOrigin;
    private float time;

    private void Awake()
    {
        fieldOrigin = XZ(transform.position) + fieldOffset;
        Transform[] pivots = treePivots.Length == 0 ? new[] { transform } : treePivots;
        trees = new TreeState[pivots.Length];
        for (int i = 0; i < pivots.Length; i++)
        {
            Transform pivot = pivots[i];
            if (pivot == null) continue;
            trees[i] = new TreeState { Pivot = pivot, RestRotation = pivot.localRotation };
        }

        grid = new WindGrid();
        ReplayGust();
    }

    [ContextMenu("Replay Gust")]
    public void ReplayGust()
    {
        if (grid == null) return;

        time = 0f;
        grid.Reset(backgroundWind);
        foreach (TreeState tree in trees)
        {
            if (tree == null || tree.Pivot == null) continue;
            tree.Lean = Vector2.zero;
            tree.LeanRate = Vector2.zero;
            tree.Pivot.localRotation = tree.RestRotation;
        }
    }

    private void FixedUpdate()
    {
        float dt = Time.fixedDeltaTime;
        grid.Step(dt, time, gustCenter, gustWidth, gustDuration, gustAcceleration);
        time += dt;

        float frequency = 2f * Mathf.PI * naturalFrequencyHz;
        float damping = 2f * dampingRatio * frequency;
        float leanLimit = maxLeanDegrees * Mathf.Deg2Rad;

        foreach (TreeState tree in trees)
        {
            if (tree == null || tree.Pivot == null) continue;

            Vector2 loadPosition = XZ(tree.Pivot.position) + loadHeight * tree.Lean;
            Vector2 wind = grid.SampleVelocity(loadPosition - fieldOrigin);
            // Small-angle approximation of the load point's horizontal velocity.
            Vector2 relativeWind = wind - loadHeight * tree.LeanRate;
            Vector2 acceleration = dragResponse * relativeWind.magnitude * relativeWind
                - damping * tree.LeanRate - frequency * frequency * tree.Lean;

            tree.LeanRate += dt * acceleration;
            tree.Lean += dt * tree.LeanRate;

            float angle = tree.Lean.magnitude;
            Vector2 direction = angle > 0.00001f ? tree.Lean / angle : Vector2.zero;
            if (angle > leanLimit)
            {
                tree.Lean = leanLimit * direction;
                tree.LeanRate -= Mathf.Max(0f, Vector2.Dot(tree.LeanRate, direction)) * direction;
                angle = leanLimit;
            }

            // For an upright tree, cross(world up, downwind) gives the bend axis.
            Vector3 axis = new Vector3(direction.y, 0f, -direction.x);
            Transform parent = tree.Pivot.parent;
            if (parent != null) axis = parent.InverseTransformDirection(axis);
            tree.Pivot.localRotation = angle > 0.00001f
                ? Quaternion.AngleAxis(angle * Mathf.Rad2Deg, axis) * tree.RestRotation
                : tree.RestRotation;
        }
    }

    private static Vector2 XZ(Vector3 position) => new Vector2(position.x, position.z);

    // Periodic MAC grid. U is stored on left faces, V on bottom faces,
    // and pressure potential at cell centres. Coordinates are relative to fieldOrigin.
    private sealed class WindGrid
    {
        private readonly float[] u = new float[GridSize * GridSize];
        private readonly float[] v = new float[GridSize * GridSize];
        private readonly float[] uStar = new float[GridSize * GridSize];
        private readonly float[] vStar = new float[GridSize * GridSize];
        private readonly float[] divergence = new float[GridSize * GridSize];
        private float[] pressure = new float[GridSize * GridSize];
        private float[] nextPressure = new float[GridSize * GridSize];

        public void Reset(Vector2 velocity)
        {
            for (int index = 0; index < u.Length; index++)
            {
                u[index] = velocity.x;
                v[index] = velocity.y;
            }
        }

        public Vector2 SampleVelocity(Vector2 metres)
        {
            Vector2 cell = metres / CellWidth;
            return new Vector2(
                Sample(u, cell.x, cell.y - 0.5f),
                Sample(v, cell.x - 0.5f, cell.y));
        }

        public void Step(float dt, float time, Vector2 center, float width, float duration, float strength)
        {
            float pulse = time < duration ? Mathf.Sin(Mathf.PI * time / duration) : 0f;
            float gustImpulse = dt * strength * pulse * pulse;
            float inverseWidthSquared = 0.5f / (width * width);

            for (int j = 0; j < GridSize; j++)
            for (int i = 0; i < GridSize; i++)
            {
                int index = Index(i, j);
                Vector2 uFace = CellWidth * new Vector2(i, j + 0.5f);
                Vector2 vFace = CellWidth * new Vector2(i + 0.5f, j);
                Vector2 uBack = (uFace - dt * SampleVelocity(uFace)) / CellWidth;
                Vector2 vBack = (vFace - dt * SampleVelocity(vFace)) / CellWidth;

                Vector2 offset = uFace - center;
                float domainWidth = GridSize * CellWidth;
                offset.x -= domainWidth * Mathf.Round(offset.x / domainWidth);
                offset.y -= domainWidth * Mathf.Round(offset.y / domainWidth);
                uStar[index] = Sample(u, uBack.x, uBack.y - 0.5f)
                    + gustImpulse * Mathf.Exp(-offset.sqrMagnitude * inverseWidthSquared);
                vStar[index] = Sample(v, vBack.x - 0.5f, vBack.y);
            }

            Project();
        }

        private void Project()
        {
            for (int j = 0; j < GridSize; j++)
            for (int i = 0; i < GridSize; i++)
            {
                int index = Index(i, j);
                divergence[index] = (uStar[Index(i + 1, j)] - uStar[index]
                    + vStar[Index(i, j + 1)] - vStar[index]) / CellWidth;
            }

            Array.Clear(pressure, 0, pressure.Length);
            for (int iteration = 0; iteration < PressureIterations; iteration++)
            {
                for (int j = 0; j < GridSize; j++)
                for (int i = 0; i < GridSize; i++)
                {
                    int index = Index(i, j);
                    float estimate = 0.25f * (pressure[Index(i - 1, j)] + pressure[Index(i + 1, j)]
                        + pressure[Index(i, j - 1)] + pressure[Index(i, j + 1)]
                        - CellWidth * CellWidth * divergence[index]);
                    nextPressure[index] = pressure[index] + (2f / 3f) * (estimate - pressure[index]);
                }

                float[] previous = pressure;
                pressure = nextPressure;
                nextPressure = previous;
            }

            for (int j = 0; j < GridSize; j++)
            for (int i = 0; i < GridSize; i++)
            {
                int index = Index(i, j);
                u[index] = uStar[index] - (pressure[index] - pressure[Index(i - 1, j)]) / CellWidth;
                v[index] = vStar[index] - (pressure[index] - pressure[Index(i, j - 1)]) / CellWidth;
            }
        }

        private static float Sample(float[] field, float x, float z)
        {
            int i = Mathf.FloorToInt(x);
            int j = Mathf.FloorToInt(z);
            float bottom = Mathf.LerpUnclamped(field[Index(i, j)], field[Index(i + 1, j)], x - i);
            float top = Mathf.LerpUnclamped(field[Index(i, j + 1)], field[Index(i + 1, j + 1)], x - i);
            return Mathf.LerpUnclamped(bottom, top, z - j);
        }

        private static int Index(int i, int j)
        {
            // GridSize is a power of two, so a mask wraps negative indices too.
            return (i & (GridSize - 1)) + GridSize * (j & (GridSize - 1));
        }
    }
}
