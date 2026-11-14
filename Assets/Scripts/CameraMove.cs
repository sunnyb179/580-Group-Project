using UnityEngine;
using System.Collections;

[DisallowMultipleComponent]
public sealed class CameraMove : MonoBehaviour
{
    [SerializeField, Min(0.1f)] private float duration = 8f;
    [SerializeField, Min(0f)] private float distance = 4f;
    [SerializeField] private Material warpMaterial;
    [SerializeField, Range(0f, 0.99f)] private float historyBlend = 0.93f;
    private RenderTexture historyFrame;

    private Vector3 startPosition;
    private Vector3 forward;
    private float elapsed;

    private void Start()
    {
        startPosition = transform.position;
        forward = transform.forward;
        if (warpMaterial != null)
            historyFrame = warpMaterial.GetTexture("_HistoryTex") as RenderTexture;

        if (historyFrame == null)
        {
            Debug.LogWarning("Check if MotionWarpMat's HistoryTex is assigned to HistoryFrame。", this);
            return;
        }

        StartCoroutine(CaptureHistory());
    }

    private void Update()
    {
        float dt = Time.deltaTime;
        if (dt <= 0f) return;

        Vector3 beforeMove = transform.position;

        elapsed += dt;
        float progress = Mathf.Clamp01(elapsed / duration);
        float offset = Mathf.SmoothStep(0f, distance, progress);
        transform.position = startPosition + forward * offset;

        float speed = Vector3.Distance(beforeMove, transform.position) / dt;

        if (warpMaterial != null)
        {
            float strength = 8f * Mathf.Clamp01(speed / 0.75f);
            warpMaterial.SetFloat("_WarpStrength", strength);
        }
    }
    private IEnumerator CaptureHistory()
    {
        var endOfFrame = new WaitForEndOfFrame();
        warpMaterial.SetFloat("_HistoryBlend", 0f);

        while (true)
        {
            yield return endOfFrame;
            ScreenCapture.CaptureScreenshotIntoRenderTexture(historyFrame);
            warpMaterial.SetFloat("_HistoryBlend", historyBlend);
        }
    }
    private void OnDisable()
    {
        if (warpMaterial != null)
            warpMaterial.SetFloat("_HistoryBlend", 0f);
    }
}
