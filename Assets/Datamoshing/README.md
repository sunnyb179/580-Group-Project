# Datamoshing camera trails

This prototype explores how camera speed affects the trails you see around foliage. Open [CameraTest](../Scenes/CameraTest.unity), set the Game view to 1920 × 1080, and press **Play**. The camera moves toward a tree over eight seconds. The leaf edges stretch most near the middle of the shot, when the camera moves fastest.

A fullscreen shader uses motion vectors to shift pixels from the previous frame and blend them into the current view. Camera speed controls how far the pixels shift. **Main Camera → Camera Move → History Blend** controls how long the trails linger. Each run starts with a fresh frame.
