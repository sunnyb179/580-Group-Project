using System.Collections.Generic;
using UnityEngine;

public sealed partial class ForestWind
{
    [Header("Automatic sway")]
    [Tooltip("Infer branch joints from disconnected woody pieces. Trees with continuous wood use whole-tree sway.")]
    [SerializeField] private bool automaticBranches = true;

    private sealed class MeshPiece
    {
        public readonly List<int> Vertices = new();
        public readonly List<Vector2Int> Edges = new();
    }

    private sealed class MeshTopology
    {
        public int[] Membership;
        public readonly List<MeshPiece> Parts = new();
    }

    private sealed class AutoBranch
    {
        public int Parent;
        public Vector3 Root;
        public Vector3 Tip;
        public Vector3 Bend;
        public Vector3 Rate;
        public Vector3 PreviousLoad;
        public Quaternion Rotation = Quaternion.identity;
        public Quaternion InheritedRotation = Quaternion.identity;
        public Vector3 PosedRoot;
    }

    private static readonly Dictionary<Mesh, MeshTopology> MeshTopologies = new();

    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.SubsystemRegistration)]
    private static void ResetSharedCaches()
    {
        Fields.Clear();
        MeshTopologies.Clear();
        SwaySources.Clear();
    }

    private readonly List<AutoBranch> autoBranches = new();

    private void BindAutomaticBranches()
    {
        if (!automaticBranches) return;

        SwayMesh reference = swayMeshes.Find(part => IsWood(part) && TopologyFor(part.Source).Parts.Count > 1);
        if (reference == null) return;

        MeshTopology topology = TopologyFor(reference.Source);
        int root = 0;
        float lowest = float.PositiveInfinity;
        for (int i = 0; i < topology.Parts.Count; i++)
        foreach (int vertex in topology.Parts[i].Vertices)
        {
            if (reference.Rest[vertex].y >= lowest) continue;
            lowest = reference.Rest[vertex].y;
            root = i;
        }

        // Attach each piece to the closest edge of the wood already joined to the trunk.
        var attached = new Dictionary<int, int> { [root] = -1 };
        while (attached.Count < topology.Parts.Count)
        {
            int next = -1;
            int parent = -1;
            float bestDistance = float.PositiveInfinity;
            Vector3 attachment = default;
            for (int i = 0; i < topology.Parts.Count; i++)
            {
                if (attached.ContainsKey(i)) continue;
                foreach (var candidate in attached)
                foreach (Vector2Int edge in topology.Parts[candidate.Key].Edges)
                foreach (int vertex in topology.Parts[i].Vertices)
                {
                    Vector3 point = reference.Rest[vertex];
                    Vector3 nearest = ClosestOnSegment(point, reference.Rest[edge.x], reference.Rest[edge.y]);
                    float distance = (point - nearest).sqrMagnitude;
                    if (distance >= bestDistance) continue;
                    bestDistance = distance;
                    attachment = point;
                    next = i;
                    parent = candidate.Value;
                }
            }

            // Unrelated props and floating details should stay on the whole-tree motion.
            if (next < 0 || bestDistance > treeHeight * treeHeight * 0.01f) break;
            List<int> vertices = topology.Parts[next].Vertices;
            Vector3 farthest = attachment;
            foreach (int vertex in vertices)
                if ((reference.Rest[vertex] - attachment).sqrMagnitude > (farthest - attachment).sqrMagnitude)
                    farthest = reference.Rest[vertex];

            Vector3 axis = (farthest - attachment).normalized;
            float length = Vector3.Distance(farthest, attachment);
            if (length < treeHeight * 0.015f)
            {
                attached.Add(next, parent);
                continue;
            }

            Vector3 baseSum = Vector3.zero;
            Vector3 tipSum = Vector3.zero;
            int baseCount = 0;
            int tipCount = 0;
            foreach (int vertex in vertices)
            {
                Vector3 point = reference.Rest[vertex];
                float along = Vector3.Dot(point - attachment, axis);
                if (along < length * 0.22f)
                {
                    baseSum += point;
                    baseCount++;
                }
                if (along > length * 0.78f)
                {
                    tipSum += point;
                    tipCount++;
                }
            }

            attached.Add(next, autoBranches.Count);
            autoBranches.Add(new AutoBranch
            {
                Parent = parent,
                Root = baseCount > 0 ? baseSum / baseCount : attachment,
                Tip = tipCount > 0 ? tipSum / tipCount : farthest
            });
        }

        if (autoBranches.Count == 0) return;
        var foliageBindings = new Dictionary<string, int>();
        foreach (SwayMesh part in swayMeshes)
        {
            part.BranchIds = new int[part.Rest.Length];
            part.BranchWeights = new float[part.Rest.Length];
            if (part == reference)
            {
                for (int i = 0; i < part.Rest.Length; i++)
                {
                    part.BranchIds[i] = attached.TryGetValue(topology.Membership[i], out int branch) ? branch : -1;
                    part.BranchWeights[i] = BranchInfluence(part.Rest[i], part.BranchIds[i]);
                }
            }
            else if (IsWood(part))
            {
                MeshTopology other = TopologyFor(part.Source);
                var sourceParts = new int[other.Parts.Count];
                for (int i = 0; i < other.Parts.Count; i++)
                {
                    // Match whole pieces so split normals and seams share the same joint.
                    Vector3 center = CenterOf(part, other.Parts[i]);
                    float best = float.PositiveInfinity;
                    int match = -1;
                    for (int j = 0; j < topology.Parts.Count; j++)
                    {
                        float distance = (center - CenterOf(reference, topology.Parts[j])).sqrMagnitude;
                        if (distance >= best) continue;
                        best = distance;
                        match = j;
                    }
                    sourceParts[i] = attached.TryGetValue(match, out int branch) ? branch : -1;
                }

                for (int i = 0; i < part.Rest.Length; i++)
                {
                    part.BranchIds[i] = sourceParts[other.Membership[i]];
                    part.BranchWeights[i] = BranchInfluence(part.Rest[i], part.BranchIds[i]);
                }
            }
            else
            {
                string meshName = part.Source.name;
                int suffix = meshName.LastIndexOf("_LOD", System.StringComparison.OrdinalIgnoreCase);
                string name = suffix < 0 ? meshName : meshName.Substring(0, suffix);
                if (!foliageBindings.TryGetValue(name, out int bestBranch))
                {
                    Vector3 center = part.ToTree.MultiplyPoint3x4(part.RestBounds.center);
                    float best = float.PositiveInfinity;
                    bestBranch = -1;
                    for (int i = 0; i < autoBranches.Count; i++)
                    {
                        float distance = (center - autoBranches[i].Tip).sqrMagnitude;
                        if (distance >= best) continue;
                        best = distance;
                        bestBranch = i;
                    }
                    foliageBindings.Add(name, bestBranch);
                }
                for (int i = 0; i < part.Rest.Length; i++)
                {
                    part.BranchIds[i] = bestBranch;
                    part.BranchWeights[i] = 1f;
                }
            }
        }
    }

    private static bool IsWood(SwayMesh part)
    {
        return part.Source.name.StartsWith("Trunk", System.StringComparison.OrdinalIgnoreCase)
            || part.Source.name.StartsWith("Branch", System.StringComparison.OrdinalIgnoreCase);
    }

    private static Vector3 CenterOf(SwayMesh mesh, MeshPiece part)
    {
        Vector3 sum = Vector3.zero;
        foreach (int vertex in part.Vertices) sum += mesh.Rest[vertex];
        return sum / part.Vertices.Count;
    }

    private float BranchInfluence(Vector3 point, int index)
    {
        if (index < 0) return 0f;
        AutoBranch branch = autoBranches[index];
        Vector3 axis = branch.Tip - branch.Root;
        float along = Vector3.Dot(point - branch.Root, axis) / Mathf.Max(0.000001f, axis.sqrMagnitude);
        return Mathf.SmoothStep(0f, 1f, Mathf.Clamp01(along * 1.4f));
    }

    private static Vector3 ClosestOnSegment(Vector3 point, Vector3 start, Vector3 end)
    {
        Vector3 edge = end - start;
        float t = Vector3.Dot(point - start, edge) / Mathf.Max(0.000001f, edge.sqrMagnitude);
        return start + Mathf.Clamp01(t) * edge;
    }

    private static MeshTopology TopologyFor(Mesh mesh)
    {
        if (MeshTopologies.TryGetValue(mesh, out MeshTopology topology)) return topology;

        Vector3[] vertices = mesh.vertices;
        var welded = new Dictionary<Vector3Int, int>();
        int[] parents = new int[vertices.Length];
        float scale = 100000f / Mathf.Max(0.01f, mesh.bounds.size.magnitude);
        for (int i = 0; i < vertices.Length; i++)
        {
            Vector3Int key = Vector3Int.RoundToInt(vertices[i] * scale);
            if (!welded.TryGetValue(key, out int first))
            {
                first = i;
                welded.Add(key, first);
            }
            parents[i] = first;
        }

        int[] triangles = mesh.triangles;
        for (int i = 0; i < triangles.Length; i += 3)
        {
            int a = FindRoot(parents, triangles[i]);
            parents[FindRoot(parents, triangles[i + 1])] = a;
            parents[FindRoot(parents, triangles[i + 2])] = a;
        }

        topology = new MeshTopology { Membership = new int[vertices.Length] };
        var parts = new Dictionary<int, int>();
        for (int i = 0; i < vertices.Length; i++)
        {
            int root = FindRoot(parents, i);
            if (!parts.TryGetValue(root, out int part))
            {
                part = topology.Parts.Count;
                parts.Add(root, part);
                topology.Parts.Add(new MeshPiece());
            }
            topology.Membership[i] = part;
            topology.Parts[part].Vertices.Add(i);
        }

        for (int i = 0; i < triangles.Length; i += 3)
        {
            int a = triangles[i];
            int b = triangles[i + 1];
            int c = triangles[i + 2];
            List<Vector2Int> edges = topology.Parts[topology.Membership[a]].Edges;
            edges.Add(new Vector2Int(a, b));
            edges.Add(new Vector2Int(b, c));
            edges.Add(new Vector2Int(c, a));
        }

        MeshTopologies.Add(mesh, topology);
        return topology;
    }

    private static int FindRoot(int[] parents, int vertex)
    {
        while (parents[vertex] != vertex)
        {
            parents[vertex] = parents[parents[vertex]];
            vertex = parents[vertex];
        }
        return vertex;
    }

    private void ResetBranchSway()
    {
        foreach (AutoBranch branch in autoBranches)
        {
            branch.Bend = Vector3.zero;
            branch.Rate = Vector3.zero;
            branch.Rotation = Quaternion.identity;
            branch.InheritedRotation = Quaternion.identity;
            branch.PosedRoot = branch.Root;
            branch.PreviousLoad = transform.TransformPoint(branch.Tip);
        }
    }

    private void StepBranchSway(float dt, float frequency, float damping, float limit)
    {
        Quaternion treeBend = BendRotation(swayBend);
        foreach (AutoBranch branch in autoBranches)
        {
            Vector3 root = branch.Root;
            Quaternion parentRotation = Quaternion.identity;
            if (branch.Parent >= 0)
            {
                AutoBranch parent = autoBranches[branch.Parent];
                Quaternion inherited = parent.InheritedRotation;
                parentRotation = inherited * Quaternion.SlerpUnclamped(Quaternion.identity, parent.Rotation,
                    BranchInfluence(root, branch.Parent));
                root = parent.PosedRoot + parentRotation * (root - parent.Root);
            }

            branch.PosedRoot = root;
            branch.InheritedRotation = parentRotation;
            Quaternion oldRotation = parentRotation * branch.Rotation;
            Vector3 tip = root + oldRotation * (branch.Tip - branch.Root);
            Vector3 acceleration = Quaternion.Inverse(parentRotation) * transform.InverseTransformDirection(
                WindTorque(transform.TransformPoint(BendTreePoint(root, treeBend)),
                    transform.TransformPoint(BendTreePoint(tip, treeBend)), ref branch.PreviousLoad, dt));
            IntegrateBend(ref branch.Bend, ref branch.Rate, acceleration * 0.65f,
                dt, frequency * 1.35f, damping * 1.35f, limit * 0.65f);
            branch.Rotation = BendRotation(branch.Bend);
        }
    }
}
