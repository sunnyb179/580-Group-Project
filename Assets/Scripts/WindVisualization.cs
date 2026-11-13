using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;

[DefaultExecutionOrder(100)]
[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
public sealed class WindVisualization : MonoBehaviour
{
    [Flags]
    public enum View
    {
        Pathlines = 1,
        Streamlines = 2,
        Streaklines = 4,
        Arrows = 8,
        Heatmap = 16
    }

    public static readonly View[] Views = { View.Pathlines, View.Streamlines, View.Streaklines, View.Arrows, View.Heatmap };
    public static readonly string[] Labels = { "Particle trails", "Streamlines", "Streaklines", "Wind arrows", "Wind speed heatmap" };

    [SerializeField] private ForestWind wind;
    [SerializeField] private Material overlayMaterial;
    [SerializeField] private View visibleViews = View.Pathlines;
    [SerializeField] private Vector2 area = new(26f, 18f);
    [SerializeField] private float traceHeight = 2.5f;
    [SerializeField, Min(1f)] private float maximumLegendSpeed = 16f;

    private const int TracerCount = 54;
    private const int HistoryLength = 32;
    private const int StreakCount = 5;
    private const int StreakLength = 28;
    private const float EmissionInterval = 0.12f;
    private static readonly Color32 Slow = new(64, 126, 255, 255);
    private static readonly Color32 Fast = new(255, 178, 65, 255);
    private static readonly Color32 TrailColor = new(169, 241, 255, 235);
    private static readonly Color32 StreamColor = new(151, 255, 179, 220);
    private static readonly Color32 StreakColor = new(244, 143, 255, 235);

    private sealed class Tracer
    {
        public readonly Vector2[] History = new Vector2[HistoryLength];
        public Vector2 Position;
        public int Head;
        public int Count;
        public int Generation;

        public void Add(Vector2 point)
        {
            Position = point;
            History[Head] = point;
            Head = (Head + 1) % History.Length;
            Count = Mathf.Min(Count + 1, History.Length);
        }
    }

    private sealed class Streak
    {
        public readonly List<Vector2> Points = new(StreakLength);
        public Vector2 Source;
    }

    private readonly Tracer[] tracers = new Tracer[TracerCount];
    private readonly Streak[] streaks = new Streak[StreakCount];
    private readonly List<Vector3> vertices = new(12000);
    private readonly List<Color32> colors = new(12000);
    private readonly List<int> indices = new(18000);
    private Mesh mesh;
    private float lastTime = -1f;
    private float lastEmission;
    private float inletDirection;
    private float traceAccumulator;
    private float meshOriginY;
    private Matrix4x4 worldToMesh;
    private Vector2 Center => new(transform.position.x, transform.position.z);
    private Vector2 HalfArea => area * 0.5f;

    public bool IsVisible(View view) => (visibleViews & view) != 0;

    public void SetVisible(View view, bool visible)
    {
        if (visible == IsVisible(view)) return;
        visibleViews = visible ? visibleViews | view : visibleViews & ~view;
        if (visible && lastTime >= 0f)
        {
            if (view == View.Pathlines) ResetTracers();
            if (view == View.Streaklines) ResetStreaks(lastTime);
        }
        if (isActiveAndEnabled) Rebuild();
    }

    private void OnEnable()
    {
        if (!Application.isPlaying || overlayMaterial == null) return;
        mesh = new Mesh { name = "Wind visualization", hideFlags = HideFlags.DontSave };
        mesh.MarkDynamic();
        GetComponent<MeshFilter>().sharedMesh = mesh;
        GetComponent<MeshRenderer>().sharedMaterial = overlayMaterial;
        lastTime = -1f;
    }

    private void OnDisable()
    {
        if (mesh == null) return;
        GetComponent<MeshFilter>().sharedMesh = null;
        Destroy(mesh);
        mesh = null;
    }

    private void FixedUpdate()
    {
        if (mesh == null || wind == null || !wind.TryGetWindTime(out float time)) return;
        if (lastTime < 0f || time < lastTime)
        {
            ResetTraces(time);
            lastTime = time;
            Rebuild();
            return;
        }
        if (time <= lastTime) return;

        float dt = time - lastTime;
        lastTime = time;
        traceAccumulator += dt;
        if (traceAccumulator < 0.033f) return;
        dt = traceAccumulator;
        traceAccumulator = 0f;

        if (IsVisible(View.Pathlines)) AdvanceTracers(dt);
        if (IsVisible(View.Streaklines)) AdvanceStreaks(dt, time);
        Rebuild();
    }

    private void ResetTraces(float time)
    {
        inletDirection = wind.SampleWind(Center).x >= 0f ? 1f : -1f;
        traceAccumulator = 0f;
        ResetTracers();
        ResetStreaks(time);
    }

    private void ResetTracers()
    {
        for (int i = 0; i < tracers.Length; i++)
        {
            tracers[i] ??= new Tracer();
            tracers[i].Generation = 0;
            SpawnTracer(tracers[i], i, true);
        }
    }

    private void ResetStreaks(float time)
    {
        lastEmission = time - EmissionInterval;
        for (int i = 0; i < streaks.Length; i++)
        {
            streaks[i] ??= new Streak();
            Streak streak = streaks[i];
            streak.Points.Clear();
            float across = Mathf.Lerp(-0.75f, 0.75f, i / (float)(streaks.Length - 1));
            streak.Source = Center + new Vector2(-inletDirection * (HalfArea.x - 0.2f), across * HalfArea.y);
        }
    }

    private void SpawnTracer(Tracer tracer, int index, bool scatter)
    {
        float lane = Mathf.Repeat(index * 0.618034f + tracer.Generation * 0.381966f, 1f);
        float depth = scatter ? Mathf.Repeat(index * 0.754878f, 1f) * 1.8f - 0.9f : -inletDirection * 0.97f;
        tracer.Head = 0;
        tracer.Count = 0;
        tracer.Add(Center + new Vector2(depth * HalfArea.x, (lane * 1.9f - 0.95f) * HalfArea.y));
        tracer.Generation++;
    }

    private bool Contains(Vector2 point)
    {
        Vector2 delta = point - Center;
        return Mathf.Abs(delta.x) <= HalfArea.x && Mathf.Abs(delta.y) <= HalfArea.y;
    }

    private Vector2 Advect(Vector2 point, float dt)
    {
        Vector2 midpoint = point + wind.SampleWind(point) * (0.5f * dt);
        return point + wind.SampleWind(midpoint) * dt;
    }

    private void AdvanceTracers(float dt)
    {
        for (int i = 0; i < tracers.Length; i++)
        {
            Tracer tracer = tracers[i];
            Vector2 next = Advect(tracer.Position, dt);
            if (Contains(next)) tracer.Add(next);
            else SpawnTracer(tracer, i, false);
        }
    }

    private void AdvanceStreaks(float dt, float time)
    {
        bool emit = time - lastEmission >= EmissionInterval;
        if (emit) lastEmission = time;
        foreach (Streak streak in streaks)
        {
            for (int i = streak.Points.Count - 1; i >= 0; i--)
            {
                Vector2 point = Advect(streak.Points[i], dt);
                if (Contains(point)) streak.Points[i] = point;
                else streak.Points.RemoveAt(i);
            }
            if (!emit) continue;
            if (streak.Points.Count == StreakLength) streak.Points.RemoveAt(0);
            streak.Points.Add(streak.Source);
        }
    }

    private void Rebuild()
    {
        if (mesh == null) return;
        vertices.Clear();
        colors.Clear();
        indices.Clear();
        meshOriginY = transform.position.y;
        worldToMesh = transform.worldToLocalMatrix;

        if (wind != null && wind.TryGetWindTime(out _))
        {
            if (IsVisible(View.Heatmap)) DrawHeatmap();
            if (IsVisible(View.Arrows)) DrawArrows();
            if (IsVisible(View.Streamlines)) DrawStreamlines();
            if (IsVisible(View.Streaklines) && streaks[0] != null) DrawStreaklines();
            if (IsVisible(View.Pathlines) && tracers[0] != null) DrawPathlines();
        }

        mesh.Clear();
        if (indices.Count == 0) return;
        mesh.SetVertices(vertices);
        mesh.SetColors(colors);
        mesh.SetTriangles(indices, 0);
    }

    private void DrawPathlines()
    {
        for (int i = 0; i < tracers.Length; i++)
        {
            Tracer tracer = tracers[i];
            float height = traceHeight + (i % 3 - 1) * 0.52f;
            int start = (tracer.Head - tracer.Count + HistoryLength) % HistoryLength;
            for (int j = 1; j < tracer.Count; j++)
            {
                Color32 color = TrailColor;
                color.a = (byte)(TrailColor.a * j / tracer.Count);
                Segment(tracer.History[(start + j - 1) % HistoryLength],
                    tracer.History[(start + j) % HistoryLength], height, 0.038f, color);
            }
            Vector2 dot = Vector2.one * 0.049f;
            Quad(tracer.Position - dot, tracer.Position + new Vector2(dot.x, -dot.y),
                tracer.Position + dot, tracer.Position + new Vector2(-dot.x, dot.y),
                height, TrailColor, TrailColor, TrailColor, TrailColor);
        }
    }

    private void DrawStreamlines()
    {
        float height = traceHeight + 0.82f;
        for (int lane = 0; lane < 9; lane++)
        {
            Vector2 point = Center + new Vector2(-inletDirection * (HalfArea.x - 0.15f),
                Mathf.Lerp(-0.9f, 0.9f, lane / 8f) * HalfArea.y);
            for (int step = 0; step < 96; step++)
            {
                Vector2 velocity = wind.SampleWind(point);
                if (velocity.sqrMagnitude < 0.0001f) break;
                Vector2 next = point + 0.32f * wind.SampleWind(point + 0.16f * velocity.normalized).normalized;
                if (!Contains(next)) break;
                if (step % 3 != 2) Segment(point, next, height, 0.034f, StreamColor);
                if (step % 15 == 10) Arrowhead(next, next - point, height, 0.24f, StreamColor);
                point = next;
            }
        }
    }

    private void DrawStreaklines()
    {
        float height = traceHeight - 0.78f;
        foreach (Streak streak in streaks)
        {
            for (int i = 1; i < streak.Points.Count; i++)
            {
                Color32 color = StreakColor;
                color.a = (byte)Mathf.Lerp(85f, StreakColor.a, i / (float)(streak.Points.Count - 1));
                Segment(streak.Points[i - 1], streak.Points[i], height, 0.065f, color);
            }
        }
    }

    private void DrawArrows()
    {
        const int columns = 13;
        const int rows = 9;
        for (int z = 0; z < rows; z++)
        for (int x = 0; x < columns; x++)
        {
            Vector2 point = GridPoint((x + 0.5f) / columns, (z + 0.5f) / rows);
            Vector2 velocity = wind.SampleWind(point);
            float speed = velocity.magnitude;
            if (speed < 0.001f) continue;
            Vector2 shaft = velocity.normalized * Mathf.Min(speed * 0.105f, 1.65f);
            Color32 color = SpeedColor(speed, 245);
            Vector2 tip = point + shaft * 0.5f;
            Segment(point - shaft * 0.5f, tip - shaft * 0.12f, 0.16f, 0.05f, color);
            Arrowhead(tip, shaft, 0.16f, Mathf.Min(shaft.magnitude * 0.45f, 0.33f), color);
        }
    }

    private void DrawHeatmap()
    {
        const int columns = 32;
        const int rows = 22;
        for (int z = 0; z <= rows; z++)
        for (int x = 0; x <= columns; x++)
        {
            Vector2 point = GridPoint(x / (float)columns, z / (float)rows);
            AddVertex(point, 0.06f, SpeedColor(wind.SampleWind(point).magnitude, 112));
            if (x == 0 || z == 0) continue;
            int d = vertices.Count - 1;
            int a = d - columns - 2;
            AddQuadIndices(a, a + 1, d, d - 1);
        }
    }

    private Vector2 GridPoint(float x, float z) => Center + Vector2.Scale(new Vector2(x - 0.5f, z - 0.5f), area);

    private Color32 SpeedColor(float speed, byte alpha)
    {
        Color32 color = Color32.Lerp(Slow, Fast, Mathf.Clamp01(speed / maximumLegendSpeed));
        color.a = alpha;
        return color;
    }

    private void Segment(Vector2 a, Vector2 b, float height, float width, Color32 color)
    {
        Vector2 direction = b - a;
        if (direction.sqrMagnitude < 0.000001f) return;
        Vector2 side = new Vector2(-direction.y, direction.x).normalized * (width * 0.5f);
        Quad(a - side, b - side, b + side, a + side, height, color, color, color, color);
    }

    private void Arrowhead(Vector2 tip, Vector2 direction, float height, float size, Color32 color)
    {
        Vector2 axis = direction.normalized * size;
        Vector2 side = new Vector2(-axis.y, axis.x) * 0.48f;
        int start = vertices.Count;
        AddVertex(tip, height, color);
        AddVertex(tip - axis - side, height, color);
        AddVertex(tip - axis + side, height, color);
        indices.Add(start);
        indices.Add(start + 1);
        indices.Add(start + 2);
    }

    private void Quad(Vector2 a, Vector2 b, Vector2 c, Vector2 d, float height, Color32 ca, Color32 cb, Color32 cc, Color32 cd)
    {
        int start = vertices.Count;
        AddVertex(a, height, ca);
        AddVertex(b, height, cb);
        AddVertex(c, height, cc);
        AddVertex(d, height, cd);
        AddQuadIndices(start, start + 1, start + 2, start + 3);
    }

    private void AddVertex(Vector2 point, float height, Color32 color)
    {
        vertices.Add(worldToMesh.MultiplyPoint3x4(new Vector3(point.x, meshOriginY + height, point.y)));
        colors.Add(color);
    }

    private void AddQuadIndices(int a, int b, int c, int d)
    {
        indices.Add(a);
        indices.Add(b);
        indices.Add(c);
        indices.Add(a);
        indices.Add(c);
        indices.Add(d);
    }

    private void OnGUI()
    {
        if (!Application.isPlaying || !isActiveAndEnabled) return;
        DrawControls(new Vector2(16f, 16f));
    }

    public void DrawControls(Vector2 position)
    {
        bool showLegend = IsVisible(View.Heatmap) || IsVisible(View.Arrows);
        float height = showLegend ? 180f : 142f;
        GUILayout.BeginArea(new Rect(position.x, position.y, 200f, height), GUI.skin.box);
        GUILayout.Label("Wind visualization");
        for (int i = 0; i < Views.Length; i++)
        {
            View view = Views[i];
            bool wasVisible = IsVisible(view);
            bool show = GUILayout.Toggle(wasVisible, Labels[i]);
            if (show != wasVisible) SetVisible(view, show);
        }
        if (showLegend) DrawLegend();
        GUILayout.EndArea();
    }

    private void DrawLegend()
    {
        Rect bar = GUILayoutUtility.GetRect(165f, 8f);
        const int segments = 20;
        Color previous = GUI.color;
        for (int i = 0; i < segments; i++)
        {
            GUI.color = SpeedColor(maximumLegendSpeed * i / (segments - 1f), 255);
            GUI.DrawTexture(new Rect(bar.x + i * bar.width / segments, bar.y, bar.width / segments + 1f, bar.height), Texture2D.whiteTexture);
        }
        GUI.color = previous;
        GUILayout.BeginHorizontal();
        GUILayout.Label("0");
        GUILayout.FlexibleSpace();
        GUILayout.Label($"{maximumLegendSpeed:g} m/s");
        GUILayout.EndHorizontal();
    }
}
