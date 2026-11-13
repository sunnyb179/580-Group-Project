using System.Collections.Generic;
using UnityEngine;

public sealed partial class ForestWind
{
    private sealed class SwayMesh
    {
        public MeshFilter Filter;
        public MeshRenderer Renderer;
        public Mesh Source;
        public Mesh Pose;
        public Matrix4x4 ToTree;
        public Matrix4x4 ToMesh;
        public Vector3[] Rest;
        public Vector3[] Vertices;
        public Vector3[] RestNormals;
        public Vector3[] Normals;
        public Vector4[] RestTangents;
        public Vector4[] Tangents;
        public float[] Weights;
        public Bounds RestBounds;
        public int[] BranchIds;
        public float[] BranchWeights;
        public int[] LeafIds;
        public LeafMotion[] Leaves;
        public Vector3 LeafSamplePoint;
    }

    private static readonly Dictionary<Mesh, Mesh> SwaySources = new();

    private readonly List<SwayMesh> swayMeshes = new();
    private Vector3 swayRoot;
    private Vector3 swayLoad;
    private Vector3 swayBend;
    private Vector3 swayRate;
    private Vector3 previousSwayLoad;
    private float treeHeight;
    private float canopyRadius;

    private bool SetupAutomaticSway()
    {
        Bounds treeBounds = default;
        bool hasBounds = false;
        foreach (MeshFilter filter in GetComponentsInChildren<MeshFilter>(true))
        {
            if (!filter.TryGetComponent(out MeshRenderer renderer) || filter.sharedMesh == null)
                continue;

            Mesh source = filter.sharedMesh;
            if (SwaySources.TryGetValue(source, out Mesh original)) source = original;
            if (!source.isReadable)
            {
                Debug.LogWarning($"Enable Read/Write on {source.name} to animate this tree mesh.", filter);
                continue;
            }

            Matrix4x4 toTree = transform.worldToLocalMatrix * filter.transform.localToWorldMatrix;
            Vector3[] vertices = source.vertices;
            if (vertices.Length == 0) continue;

            var part = new SwayMesh
            {
                Filter = filter,
                Renderer = renderer,
                Source = source,
                Pose = Instantiate(source),
                ToTree = toTree,
                ToMesh = toTree.inverse,
                Rest = vertices,
                Vertices = new Vector3[vertices.Length],
                RestNormals = source.normals,
                RestTangents = source.tangents,
                Weights = new float[vertices.Length],
                RestBounds = source.bounds
            };
            part.Normals = new Vector3[part.RestNormals.Length];
            part.Tangents = new Vector4[part.RestTangents.Length];
            part.Pose.name = source.name + " (ForestWind Sway)";
            part.Pose.MarkDynamic();
            filter.sharedMesh = part.Pose;
            SwaySources.Add(part.Pose, source);
            swayMeshes.Add(part);

            for (int i = 0; i < vertices.Length; i++)
            {
                vertices[i] = toTree.MultiplyPoint3x4(vertices[i]);
                if (!hasBounds)
                {
                    treeBounds = new Bounds(vertices[i], Vector3.zero);
                    hasBounds = true;
                }
                treeBounds.Encapsulate(vertices[i]);
            }
        }

        if (!hasBounds) return false;

        swayRoot = new Vector3(0f, treeBounds.min.y, 0f);
        treeHeight = Mathf.Max(0.01f, treeBounds.size.y);
        swayLoad = swayRoot + Vector3.up * (0.72f * treeHeight);
        canopyRadius = Mathf.Max(0.01f, Mathf.Max(treeBounds.extents.x, treeBounds.extents.z));
        BindAutomaticBranches();
        foreach (SwayMesh part in swayMeshes)
        {
            for (int i = 0; i < part.Rest.Length; i++)
            {
                part.Weights[i] = WholeTreeWeight(part.Rest[i]);
            }

            float flutterMargin = BindLeafFlutter(part);

            // Keep the rest box enlarged for visibility checks even when a renderer is offscreen.
            Bounds bounds = part.RestBounds;
            float maxRotation = maxLeanDegrees * Mathf.Deg2Rad;
            float stretch = Mathf.Max(part.ToMesh.MultiplyVector(Vector3.right).magnitude,
                Mathf.Max(part.ToMesh.MultiplyVector(Vector3.up).magnitude, part.ToMesh.MultiplyVector(Vector3.forward).magnitude));
            float swayMargin = (treeHeight + canopyRadius * 2f) * maxRotation * (autoBranches.Count > 0 ? 2f : 1f);
            bounds.Expand(2f * (swayMargin + flutterMargin) * stretch);
            part.Pose.bounds = bounds;
        }

        return true;
    }

    private void ResetAutomaticSway()
    {
        swayBend = Vector3.zero;
        swayRate = Vector3.zero;
        previousSwayLoad = transform.TransformPoint(swayLoad);
        ResetBranchSway();
        ApplySway(false);
    }

    private void StepAutomaticSway(float dt, float frequency, float damping, float limit)
    {
        if (swayMeshes.Count == 0) return;
        Vector3 load = swayRoot + BendRotation(swayBend) * (swayLoad - swayRoot);
        Vector3 acceleration = transform.InverseTransformDirection(
            WindTorque(transform.TransformPoint(swayRoot), transform.TransformPoint(load), ref previousSwayLoad, dt));
        // The canopy load is distributed over the whole tree. Scale torque by its height.
        IntegrateBend(ref swayBend, ref swayRate, acceleration / Mathf.Max(1f, treeHeight),
            dt, frequency, damping, limit);
        StepBranchSway(dt, frequency, damping, limit);
    }

    private float WholeTreeWeight(Vector3 position)
    {
        Vector3 point = position - swayRoot;
        float height = Mathf.Clamp01(point.y / treeHeight);
        float radial = Mathf.Clamp01(new Vector2(point.x, point.z).magnitude / canopyRadius);
        return height * (0.75f + 0.25f * radial);
    }

    private Vector3 BendTreePoint(Vector3 point, Quaternion bend)
    {
        return swayRoot + Quaternion.SlerpUnclamped(Quaternion.identity, bend, WholeTreeWeight(point)) * (point - swayRoot);
    }

    private void LateUpdate()
    {
        ApplySway(true);
    }

    private void ApplySway(bool visibleOnly)
    {
        Quaternion bend = BendRotation(swayBend);
        foreach (SwayMesh part in swayMeshes)
        {
            if (part.Filter == null || part.Renderer == null || visibleOnly && !part.Renderer.isVisible)
                continue;

            UpdateLeafFlutter(part, visibleOnly, bend);
            Matrix4x4 normalToTree = part.ToMesh.transpose;
            Matrix4x4 normalToMesh = part.ToTree.transpose;
            for (int i = 0; i < part.Rest.Length; i++)
            {
                Quaternion rotation = Quaternion.SlerpUnclamped(Quaternion.identity, bend, part.Weights[i]);
                Vector3 rest = part.Rest[i];
                Quaternion leafRotation = Quaternion.identity;
                if (part.LeafIds != null && part.LeafIds[i] >= 0)
                {
                    LeafMotion leaf = part.Leaves[part.LeafIds[i]];
                    leafRotation = leaf.Rotation;
                    rest = leaf.Pivot + leafRotation * (rest - leaf.Pivot);
                }
                Quaternion branchRotation = Quaternion.identity;
                if (part.BranchIds != null && part.BranchIds[i] >= 0)
                {
                    AutoBranch branch = autoBranches[part.BranchIds[i]];
                    branchRotation = branch.InheritedRotation * Quaternion.SlerpUnclamped(Quaternion.identity, branch.Rotation, part.BranchWeights[i]);
                    rest = branch.PosedRoot + branchRotation * (rest - branch.Root);
                }
                Vector3 point = swayRoot + rotation * (rest - swayRoot);
                rotation = rotation * branchRotation * leafRotation;
                part.Vertices[i] = part.ToMesh.MultiplyPoint3x4(point);

                if (i < part.Normals.Length)
                    part.Normals[i] = normalToMesh.MultiplyVector(rotation * normalToTree.MultiplyVector(part.RestNormals[i])).normalized;

                if (i < part.Tangents.Length)
                {
                    Vector4 tangent = part.RestTangents[i];
                    Vector3 direction = part.ToMesh.MultiplyVector(rotation * part.ToTree.MultiplyVector(tangent)).normalized;
                    part.Tangents[i] = new Vector4(direction.x, direction.y, direction.z, tangent.w);
                }
            }

            part.Pose.SetVertices(part.Vertices, 0, part.Vertices.Length, UnityEngine.Rendering.MeshUpdateFlags.DontRecalculateBounds);
            if (part.Normals.Length > 0) part.Pose.normals = part.Normals;
            if (part.Tangents.Length > 0) part.Pose.tangents = part.Tangents;
        }
    }

    private void DestroyAutomaticSway()
    {
        foreach (SwayMesh part in swayMeshes)
        {
            if (part.Filter != null && part.Filter.sharedMesh == part.Pose)
                part.Filter.sharedMesh = part.Source;
            SwaySources.Remove(part.Pose);
            Destroy(part.Pose);
        }
    }
}
