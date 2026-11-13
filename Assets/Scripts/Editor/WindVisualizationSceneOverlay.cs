using UnityEditor;
using UnityEngine;

[InitializeOnLoad]
internal static class WindVisualizationSceneOverlay
{
    static WindVisualizationSceneOverlay()
    {
        SceneView.duringSceneGui += Draw;
    }

    private static void Draw(SceneView view)
    {
        if (!EditorApplication.isPlaying) return;

        WindVisualization overlay = Object.FindFirstObjectByType<WindVisualization>();
        if (overlay == null || !overlay.isActiveAndEnabled) return;

        Handles.BeginGUI();
        overlay.DrawControls(new Vector2(56f, 44f));
        Handles.EndGUI();
    }
}
