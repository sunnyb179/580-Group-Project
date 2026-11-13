using System;
using System.Collections.Generic;
using UnityEngine;

public sealed partial class ForestWind
{
    [Tooltip("Maximum independent flutter angle for disconnected leaf cards, in degrees. Zero disables flutter.")]
    [SerializeField, Range(0f, 30f)] private float leafFlutterDegrees = 16f;

    private sealed class LeafMotion
    {
        public Vector3 Pivot;
        public Vector3 Axis;
        public Vector3 Normal;
        public float Phase;
        public float Frequency;
        public Quaternion Rotation = Quaternion.identity;
    }

    private float BindLeafFlutter(SwayMesh part)
    {
        if (!part.Source.name.StartsWith("Leaf", StringComparison.OrdinalIgnoreCase)
            && !part.Source.name.StartsWith("Foliage", StringComparison.OrdinalIgnoreCase))
            return 0f;

        MeshTopology topology = TopologyFor(part.Source);
        if (topology.Parts.Count < 2) return 0f;

        part.LeafIds = new int[part.Rest.Length];
        Array.Fill(part.LeafIds, -1);
        part.LeafSamplePoint = part.ToTree.MultiplyPoint3x4(part.RestBounds.center);
        Vector3 attachment = swayRoot;
        if (part.BranchIds != null && part.BranchIds[0] >= 0)
            attachment = autoBranches[part.BranchIds[0]].Tip;

        var leaves = new List<LeafMotion>(topology.Parts.Count);
        float maxRadius = 0f;
        foreach (MeshPiece piece in topology.Parts)
        {
            // A leaf card is a small disconnected polygon, often with duplicated UV seam vertices.
            if (piece.Vertices.Count < 3 || piece.Vertices.Count > 16 || piece.Edges.Count < 3)
                continue;

            Vector3 center = CenterOf(part, piece);
            int baseIndex = piece.Vertices[0];
            foreach (int vertex in piece.Vertices)
                if ((part.Rest[vertex] - attachment).sqrMagnitude < (part.Rest[baseIndex] - attachment).sqrMagnitude)
                    baseIndex = vertex;

            Vector3 pivot = part.Rest[baseIndex];
            Vector3 normal = Vector3.zero;
            for (int i = 0; i < piece.Edges.Count; i += 3)
            {
                Vector2Int edge = piece.Edges[i];
                Vector3 start = part.Rest[edge.x];
                normal = Vector3.Cross(part.Rest[edge.y] - start, part.Rest[piece.Edges[i + 1].y] - start);
                if (normal.sqrMagnitude > 0.00000001f) break;
            }

            Vector3 axis = Vector3.Cross(normal.normalized, center - pivot).normalized;
            if (axis == Vector3.zero) continue;

            foreach (int vertex in piece.Vertices)
            {
                part.LeafIds[vertex] = leaves.Count;
                maxRadius = Mathf.Max(maxRadius, Vector3.Distance(part.Rest[vertex], pivot));
            }

            float seed = Vector3.Dot(center, new Vector3(12.9898f, 78.233f, 37.719f));
            leaves.Add(new LeafMotion
            {
                Pivot = pivot,
                Axis = axis,
                Normal = normal.normalized,
                Phase = seed,
                Frequency = 2.1f + 1.8f * Mathf.Repeat(seed * 0.31831f, 1f)
            });
        }

        part.Leaves = leaves.ToArray();
        return maxRadius * 30f * Mathf.Deg2Rad;
    }

    private void UpdateLeafFlutter(SwayMesh part, bool animate, Quaternion bend)
    {
        if (part.Leaves == null || part.Leaves.Length == 0) return;

        Vector3 localWind = Vector3.zero;
        if (animate && field != null && leafFlutterDegrees > 0f)
        {
            Vector3 samplePoint = part.LeafSamplePoint;
            if (part.BranchIds != null && part.BranchIds[0] >= 0)
            {
                AutoBranch branch = autoBranches[part.BranchIds[0]];
                Quaternion rotation = branch.InheritedRotation * branch.Rotation;
                samplePoint = branch.PosedRoot + rotation * (samplePoint - branch.Root);
            }

            Vector2 wind = field.Sample(XZ(transform.TransformPoint(BendTreePoint(samplePoint, bend))));
            localWind = transform.InverseTransformDirection(new Vector3(wind.x, 0f, wind.y));
        }

        float strength = Mathf.Clamp01(localWind.magnitude / 8f) * leafFlutterDegrees;
        float phaseTime = field == null ? 0f : field.Time * (2f * Mathf.PI);
        Vector3 direction = localWind.normalized;
        foreach (LeafMotion leaf in part.Leaves)
        {
            float exposure = Mathf.Abs(Vector3.Dot(leaf.Normal, direction));
            float phase = phaseTime * leaf.Frequency + leaf.Phase;
            float angle = strength * (0.35f + 0.65f * exposure) * Mathf.Sin(phase);
            leaf.Rotation = Quaternion.AngleAxis(angle, leaf.Axis);
        }
    }
}
