using System;
using System.Collections.Generic;
using UnityEngine;

[DisallowMultipleComponent]
public sealed partial class ForestWind : MonoBehaviour
{
    private const int GridSize = 64;
    private const float CellWidth = 2f;
    private const int PressureIterations = 40;

    [Serializable]
    private sealed class Branch
    {
        [Tooltip("Child joint at the branch base. Its descendants should be only this branch and its foliage.")]
        public Transform pivot;
        [Tooltip("Place this marker near the foliage centre and parent it under the branch pivot.")]
        public Transform loadPoint;
    }

    [Header("Single mesh branch")]
    [Tooltip("Optional. Assign the tree MeshFilter to deform part of a single mesh.")]
    [SerializeField] private MeshFilter singleMesh;
    [Tooltip("Place this marker where the branch meets the trunk.")]
    [SerializeField] private Transform branchBase;
    [Tooltip("Place this marker near the outer end of the branch.")]
    [SerializeField] private Transform branchTip;
    [Tooltip("Outer radius of the branch influence region, in world metres.")]
    [SerializeField, Min(0.01f)] private float branchRadius = 0.6f;

    [Header("Separate branches")]
    [SerializeField] private Branch[] branches = Array.Empty<Branch>();
    [Tooltip("Offset from this object to the lower X/Z corner of the wind field, in world metres.")]
    [SerializeField] private Vector2 fieldOffset = new Vector2(-44f, -64f);
    [SerializeField] private Vector2 backgroundWind = new Vector2(8f, 0f);
    [SerializeField] private Vector2 gustCenter = new Vector2(28f, 64f);
    [SerializeField, Min(0f)] private float gustAcceleration = 12f;
    [SerializeField, Min(0.1f)] private float gustWidth = 6f;
    [SerializeField, Min(0.1f)] private float gustDuration = 1f;

    [Header("Branch response")]
    [Tooltip("Effective drag torque per unit inertia and squared relative wind speed.")]
    [SerializeField, Min(0f)] private float dragResponse = 0.035f;
    [SerializeField, Range(0.2f, 3f)] private float naturalFrequencyHz = 0.65f;
    [SerializeField, Range(0.05f, 1f)] private float dampingRatio = 0.4f;
    [SerializeField, Range(1f, 35f)] private float maxLeanDegrees = 20f;

    private sealed class BranchState
    {
        public Branch Binding;
        public Quaternion RestRotation;
        public Quaternion NextRotation;
        public Vector3 PreviousLoadPosition;
        public Vector3 Bend;
        public Vector3 BendRate;
    }

    private BranchState[] branchStates;
    private static readonly Dictionary<int, SharedField> Fields = new();

    private sealed class SharedField
    {
        public readonly WindGrid Grid = new();
        public Vector2 Origin;
        public Vector2 Background;
        public Vector2 GustCenter;
        public float GustWidth;
        public float GustDuration;
        public float GustAcceleration;
        public float Time;
        public double LastStep = double.NegativeInfinity;
        public int Users;

        public void Step(float dt)
        {
            if (LastStep == UnityEngine.Time.fixedTimeAsDouble) return;
            LastStep = UnityEngine.Time.fixedTimeAsDouble;
            Grid.Step(dt, Time, GustCenter, GustWidth, GustDuration, GustAcceleration);
            Time += dt;
        }

        public Vector2 Sample(Vector2 position)
        {
            Vector2 point = position - Origin;
            // Two traveling, transverse gusts keep the canopy moving in steady wind.
            float primary = Mathf.Sin(0.16f * point.x + 0.11f * point.y - 1.35f * Time);
            float secondary = Mathf.Sin(0.09f * point.x - 0.21f * point.y - 2.1f * Time);
            Vector2 variation = new Vector2(0.11f, -0.16f).normalized * (0.23f * primary)
                + new Vector2(0.21f, 0.09f).normalized * (0.09f * secondary);
            return Grid.SampleVelocity(point) + Background.magnitude * variation;
        }

        public void Reset()
        {
            Time = 0f;
            Grid.Reset(Background);
        }
    }

    internal bool TryGetWindTime(out float time)
    {
        time = field?.Time ?? 0f;
        return field != null;
    }

    internal Vector2 SampleWind(Vector2 position) => field?.Sample(position) ?? Vector2.zero;

    private SharedField field;
    private int fieldScene;
    private Mesh originalMesh;
    private Mesh animatedMesh;
    private Vector3 meshPivot;
    private Vector3 meshTip;
    private Vector3[] restVertices;
    private Vector3[] posedVertices;
    private Vector3[] restNormals;
    private Vector3[] posedNormals;
    private Vector4[] restTangents;
    private Vector4[] posedTangents;
    private float[] meshWeights;
    private Vector3 meshBend;
    private Vector3 meshBendRate;
    private Vector3 previousMeshLoadPosition;

    private void Awake()
    {
        var validBranches = new List<BranchState>();
        var assignedPivots = new HashSet<Transform>();
        foreach (Branch branch in branches)
        {
            if (branch == null || branch.pivot == null || branch.loadPoint == null) continue;
            if (branch.pivot.parent == null || transform.IsChildOf(branch.pivot)) continue;
            if (branch.loadPoint == branch.pivot || !branch.loadPoint.IsChildOf(branch.pivot)) continue;
            if (!assignedPivots.Add(branch.pivot)) continue;

            validBranches.Add(new BranchState
            {
                Binding = branch,
                RestRotation = branch.pivot.localRotation
            });
        }

        branchStates = validBranches.ToArray();
        SetupSingleMesh();
        if (branchStates.Length == 0 && animatedMesh == null && !SetupAutomaticSway())
        {
            Debug.LogWarning("ForestWind found no readable tree meshes or valid branch bindings.", this);
            return;
        }

        fieldScene = gameObject.scene.handle;
        if (!Fields.TryGetValue(fieldScene, out field))
        {
            field = new SharedField
            {
                Origin = XZ(transform.position) + fieldOffset,
                Background = backgroundWind,
                GustCenter = gustCenter,
                GustWidth = gustWidth,
                GustDuration = gustDuration,
                GustAcceleration = gustAcceleration
            };
            field.Reset();
            Fields.Add(fieldScene, field);
        }

        field.Users++;
        ResetPose();
    }

    [ContextMenu("Replay Gust")]
    public void ReplayGust()
    {
        if (field == null) return;
        field.Reset();
        ResetPose();
    }

    private void ResetPose()
    {
        ResetAutomaticSway();
        RestoreBranches();
        if (animatedMesh != null && singleMesh != null)
        {
            meshBend = Vector3.zero;
            meshBendRate = Vector3.zero;
            previousMeshLoadPosition = singleMesh.transform.TransformPoint(meshTip);
            ApplyMeshPose(Quaternion.identity);
        }
        foreach (BranchState branch in branchStates)
        {
            if (branch.Binding.loadPoint == null) continue;
            branch.Bend = Vector3.zero;
            branch.BendRate = Vector3.zero;
            branch.PreviousLoadPosition = branch.Binding.loadPoint.position;
        }
    }

    private void OnDisable()
    {
        if (field != null) ResetPose();
    }

    private void OnDestroy()
    {
        DestroyAutomaticSway();
        if (field != null && --field.Users == 0) Fields.Remove(fieldScene);
        if (animatedMesh == null) return;
        if (singleMesh != null && singleMesh.sharedMesh == animatedMesh)
            singleMesh.sharedMesh = originalMesh;
        Destroy(animatedMesh);
    }

    private void SetupSingleMesh()
    {
        if (singleMesh == null) return;
        if (branchBase == null || branchTip == null || singleMesh.sharedMesh == null)
        {
            Debug.LogWarning("Assign Single Mesh, Branch Base and Branch Tip.", this);
            return;
        }

        originalMesh = singleMesh.sharedMesh;
        if (!originalMesh.isReadable)
        {
            Debug.LogWarning("Enable Read/Write in the tree model's import settings, then click Apply.", this);
            return;
        }

        Transform meshTransform = singleMesh.transform;
        meshPivot = meshTransform.InverseTransformPoint(branchBase.position);
        meshTip = meshTransform.InverseTransformPoint(branchTip.position);
        Vector3 segment = branchTip.position - branchBase.position;
        if (segment.sqrMagnitude < 0.0001f)
        {
            Debug.LogWarning("Move Branch Tip away from Branch Base.", this);
            return;
        }

        restVertices = originalMesh.vertices;
        meshWeights = new float[restVertices.Length];
        bool hasAffectedVertices = false;
        for (int i = 0; i < restVertices.Length; i++)
        {
            Vector3 offset = meshTransform.TransformPoint(restVertices[i]) - branchBase.position;
            meshWeights[i] = BranchWeight(offset, segment, branchRadius);
            hasAffectedVertices |= meshWeights[i] > 0f;
        }

        if (!hasAffectedVertices)
        {
            Debug.LogWarning("The branch region contains no vertices. Check the markers and Branch Radius.", this);
            return;
        }

        posedVertices = new Vector3[restVertices.Length];
        restNormals = originalMesh.normals;
        posedNormals = new Vector3[restNormals.Length];
        restTangents = originalMesh.tangents;
        posedTangents = new Vector4[restTangents.Length];
        animatedMesh = Instantiate(originalMesh);
        animatedMesh.name = originalMesh.name + " (ForestWind)";
        animatedMesh.MarkDynamic();
        singleMesh.sharedMesh = animatedMesh;
    }

    private static float BranchWeight(Vector3 offset, Vector3 segment, float radius)
    {
        float along = Vector3.Dot(offset, segment) / segment.sqrMagnitude;
        if (along <= 0f) return 0f;

        float distance = (offset - Mathf.Min(along, 1f) * segment).magnitude;
        float radialWeight = Mathf.SmoothStep(1f, 0f, distance / radius);
        return radialWeight * Mathf.SmoothStep(0f, 1f, along);
    }

    private void StepSingleMesh(float dt, float frequency, float damping, float bendLimit)
    {
        if (animatedMesh == null || singleMesh == null) return;

        Transform meshTransform = singleMesh.transform;
        Quaternion rotation = BendRotation(meshBend);
        Vector3 pivotWorld = meshTransform.TransformPoint(meshPivot);
        Vector3 loadPosition = meshTransform.TransformPoint(meshPivot + rotation * (meshTip - meshPivot));
        Vector3 acceleration = meshTransform.InverseTransformDirection(
            WindTorque(pivotWorld, loadPosition, ref previousMeshLoadPosition, dt));
        IntegrateBend(ref meshBend, ref meshBendRate, acceleration, dt, frequency, damping, bendLimit);
        ApplyMeshPose(BendRotation(meshBend));
    }

    private void ApplyMeshPose(Quaternion rotation)
    {
        for (int i = 0; i < restVertices.Length; i++)
        {
            if (meshWeights[i] == 0f)
            {
                posedVertices[i] = restVertices[i];
                if (i < restNormals.Length) posedNormals[i] = restNormals[i];
                if (i < restTangents.Length) posedTangents[i] = restTangents[i];
                continue;
            }

            Quaternion weighted = Quaternion.Slerp(Quaternion.identity, rotation, meshWeights[i]);
            posedVertices[i] = meshPivot + weighted * (restVertices[i] - meshPivot);
            if (i < restNormals.Length)
                posedNormals[i] = weighted * restNormals[i];
            if (i < restTangents.Length)
            {
                Vector4 tangent = restTangents[i];
                Vector3 direction = weighted * new Vector3(tangent.x, tangent.y, tangent.z);
                posedTangents[i] = new Vector4(direction.x, direction.y, direction.z, tangent.w);
            }
        }

        animatedMesh.vertices = posedVertices;
        if (posedNormals.Length > 0) animatedMesh.normals = posedNormals;
        if (posedTangents.Length > 0) animatedMesh.tangents = posedTangents;
        animatedMesh.RecalculateBounds();
    }

    private void RestoreBranches()
    {
        foreach (BranchState branch in branchStates)
        {
            if (branch.Binding.pivot != null)
                branch.Binding.pivot.localRotation = branch.RestRotation;
        }
    }

    private void FixedUpdate()
    {
        if (field == null) return;

        float dt = Time.fixedDeltaTime;
        field.Step(dt);

        float frequency = 2f * Mathf.PI * naturalFrequencyHz;
        float damping = 2f * dampingRatio * frequency;
        float bendLimit = maxLeanDegrees * Mathf.Deg2Rad;

        foreach (BranchState branch in branchStates)
        {
            Transform pivot = branch.Binding.pivot;
            Transform loadPoint = branch.Binding.loadPoint;
            if (pivot == null || loadPoint == null || pivot.parent == null) continue;

            Vector3 acceleration = pivot.parent.InverseTransformDirection(
                WindTorque(pivot.position, loadPoint.position, ref branch.PreviousLoadPosition, dt));
            IntegrateBend(ref branch.Bend, ref branch.BendRate, acceleration, dt, frequency, damping, bendLimit);
            branch.NextRotation = BendRotation(branch.Bend) * branch.RestRotation;
        }

        StepSingleMesh(dt, frequency, damping, bendLimit);
        StepAutomaticSway(dt, frequency, damping, bendLimit);

        // Sample all branches before changing any transform, including parent joints.
        foreach (BranchState branch in branchStates)
        {
            Transform pivot = branch.Binding.pivot;
            if (pivot != null && branch.Binding.loadPoint != null && pivot.parent != null)
                pivot.localRotation = branch.NextRotation;
        }
    }

    private Vector3 WindTorque(Vector3 pivot, Vector3 loadPosition, ref Vector3 previousLoad, float dt)
    {
        Vector2 wind = field.Sample(XZ(loadPosition));
        Vector3 loadVelocity = (loadPosition - previousLoad) / dt;
        previousLoad = loadPosition;
        Vector3 relativeWind = new Vector3(wind.x, 0f, wind.y) - loadVelocity;
        return Vector3.Cross(loadPosition - pivot, relativeWind) * (dragResponse * relativeWind.magnitude);
    }

    private static void IntegrateBend(ref Vector3 bend, ref Vector3 bendRate, Vector3 acceleration,
        float dt, float frequency, float damping, float limit)
    {
        bendRate += dt * (acceleration - damping * bendRate - frequency * frequency * bend);
        bend += dt * bendRate;

        float angle = bend.magnitude;
        if (angle <= limit) return;

        Vector3 axis = bend / angle;
        bend = limit * axis;
        bendRate -= Mathf.Max(0f, Vector3.Dot(bendRate, axis)) * axis;
    }

    private static Quaternion BendRotation(Vector3 bend)
    {
        float angle = bend.magnitude;
        return angle > 0.00001f
            ? Quaternion.AngleAxis(angle * Mathf.Rad2Deg, bend / angle)
            : Quaternion.identity;
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
