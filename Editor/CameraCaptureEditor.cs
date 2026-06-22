using UnityEngine;
using UnityEditor;
using System.IO;
using System.Globalization;
using System;

using Unity.EditorCoroutines.Editor;
using System.Collections;
using System.Collections.Generic;

public class CameraCaptureEditor : EditorWindow
{

    private int tabIndex = 0;

    private Camera cameraToUse;
    private Transform target;
    private int numRings = 4;
    private int viewsPerRing = 20;
    private float radius = 5f;
    private float height = 1.5f;
    private string outputFolder = "Output";
    private bool ShowGrid = true;
    private EditorCoroutine captureCoroutine;
    bool cancel = false;
    bool isRunning = false;
    private GameObject gizmoViewer;
    public GameObject gizmoViewerPrefab;
    bool runtimeAnim = false;
    bool TrainPostShot = false;

    private int fbs = 25;
    private float duration = 1;
    private string PostShotInstallFolder = @"C:\Program Files\Jawset Postshot\bin\postshot-cli.exe";
    private int trainStep = 5;

    private int outputFormatIndex = 0;
    private string outputFormat = "psht";

    private int profileIndex = 0;
    private string profile = "Splat3";

    int w = 1920;
    int h = 1080;
    int rays = 500;
    [MenuItem("Tools/Gaussian Splatting/Capture + COLMAP")]
    public static void ShowWindow()
    {
        GetWindow<CameraCaptureEditor>("Capture + COLMAP");
    }

    private void OnGUI()
    {

        GUILayout.Space(10);
        cameraToUse = (Camera)EditorGUILayout.ObjectField("Camera", cameraToUse, typeof(Camera), true);
        w = EditorGUILayout.IntField("Width (px)", w);
        h = EditorGUILayout.IntField("Height (px)", h);
        GUILayout.Space(10);
        rays = EditorGUILayout.IntField("PointCloud/View", rays);
        GUILayout.Space(10);
        runtimeAnim = EditorGUILayout.Toggle("Capture Runtime", runtimeAnim);

        if (runtimeAnim)
        {
            fbs = EditorGUILayout.IntField("Frames/Second", fbs);
            duration = EditorGUILayout.FloatField("Duration", duration);
        }

        TrainPostShot = EditorGUILayout.Toggle("Train On PostShot", TrainPostShot);
        if (TrainPostShot)
        {
            PostShotInstallFolder = EditorGUILayout.TextField("PostShot Install Folder", PostShotInstallFolder);
            trainStep = EditorGUILayout.IntField("Training Steps", trainStep);

            GUILayout.Label("Output Format", EditorStyles.boldLabel);
            outputFormatIndex = GUILayout.Toolbar(outputFormatIndex, new string[] { "PSHT", "PLY" });
            outputFormat = outputFormatIndex == 0 ? "psht" : "ply";
            GUILayout.Label("Training Profile", EditorStyles.boldLabel);
            profileIndex = GUILayout.Toolbar(profileIndex, new string[] { "Splat3", "MCMC", "ADC" });
            profile = new string[] { "Splat3", "MCMC", "ADC" }[profileIndex];
        }

        EditorGUILayout.BeginHorizontal();
        GUILayout.Label("Ouptput folder :", EditorStyles.label);

        outputFolder = EditorGUILayout.TextField(outputFolder);

        if (GUILayout.Button("Choose...", GUILayout.MaxWidth(80)))
        {
            string selected = EditorUtility.OpenFolderPanel("Choose an output folder", "", "");
            if (!string.IsNullOrEmpty(selected))
                outputFolder = selected;
        }
        EditorGUILayout.EndHorizontal();

        GUILayout.Space(10);
        tabIndex = GUILayout.Toolbar(tabIndex, new string[] { "Dome Capture", "Volume Capture" });
      

        if (tabIndex == 0)
            DrawSphericalCaptureUI();
        else
            DrawVolumeCaptureUI();


        GUILayout.Space(20);



        if (isRunning)
            if (GUILayout.Button("Cancel"))
                {
                cancel = true;
                }


    }

    private static Quaternion QuaternionFromMatrixRH(Matrix4x4 m)
    {
        Quaternion q = new Quaternion();
        float tr = m.m00 + m.m11 + m.m22;

        if (tr > 0)
        {
            float S = Mathf.Sqrt(tr + 1.0f) * 2f; 
            q.w = 0.25f * S;
            q.x = (m.m21 - m.m12) / S;
            q.y = (m.m02 - m.m20) / S;
            q.z = (m.m10 - m.m01) / S;
        }
        else if ((m.m00 > m.m11) && (m.m00 > m.m22))
        {
            float S = Mathf.Sqrt(1.0f + m.m00 - m.m11 - m.m22) * 2f; 
            q.w = (m.m21 - m.m12) / S;
            q.x = 0.25f * S;
            q.y = (m.m01 + m.m10) / S;
            q.z = (m.m02 + m.m20) / S;
        }
        else if (m.m11 > m.m22)
        {
            float S = Mathf.Sqrt(1.0f + m.m11 - m.m00 - m.m22) * 2f; 
            q.w = (m.m02 - m.m20) / S;
            q.x = (m.m01 + m.m10) / S;
            q.y = 0.25f * S;
            q.z = (m.m12 + m.m21) / S;
        }
        else
        {
            float S = Mathf.Sqrt(1.0f + m.m22 - m.m00 - m.m11) * 2f; 
            q.w = (m.m10 - m.m01) / S;
            q.x = (m.m02 + m.m20) / S;
            q.y = (m.m12 + m.m21) / S;
            q.z = 0.25f * S;
        }
        return q;
    }

    // Convert Unity world coordinates to the COLMAP/OpenCV world used by this exporter.
    // Unity:  X right, Y up,   Z forward
    // COLMAP: X right, Y down, Z forward
    private static Vector3 UnityToColmapPoint(Vector3 p)
    {
        return new Vector3(p.x, -p.y, p.z);
    }

    private static Vector3 UnityToColmapDirection(Vector3 v)
    {
        return new Vector3(v.x, -v.y, v.z);
    }

    private static Quaternion NormalizeQuaternion(Quaternion q)
    {
        float mag = Mathf.Sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
        if (mag <= Mathf.Epsilon)
            return Quaternion.identity;
        float inv = 1.0f / mag;
        return new Quaternion(q.x * inv, q.y * inv, q.z * inv, q.w * inv);
    }

    private static void GetColmapPoseFromUnityCamera(Camera cam, out Quaternion q, out Vector3 t)
    {
        Vector3 c = UnityToColmapPoint(cam.transform.position);

        // Rows of COLMAP/OpenCV world-to-camera rotation.
        // X camera axis is right, Y camera axis is down, Z camera axis is forward.
        Vector3 cvRight = UnityToColmapDirection(cam.transform.right).normalized;
        Vector3 cvDown = UnityToColmapDirection(-cam.transform.up).normalized;
        Vector3 cvForward = UnityToColmapDirection(cam.transform.forward).normalized;

        Matrix4x4 R_w2c = Matrix4x4.identity;
        R_w2c.SetRow(0, new Vector4(cvRight.x, cvRight.y, cvRight.z, 0));
        R_w2c.SetRow(1, new Vector4(cvDown.x, cvDown.y, cvDown.z, 0));
        R_w2c.SetRow(2, new Vector4(cvForward.x, cvForward.y, cvForward.z, 0));
        R_w2c.SetRow(3, new Vector4(0, 0, 0, 1));

        q = NormalizeQuaternion(QuaternionFromMatrixRH(R_w2c));
        t = R_w2c.MultiplyVector(-c); // COLMAP: X_cam = R * X_world + t, so t = -R * C
    }


    private void RunPostshotBatch()
    {
        Debug.Log("Launching PostShot Training");
        string postshotPath = PostShotInstallFolder;

        if (!Directory.Exists(outputFolder))
        {
            Debug.LogError("Le dossier de sortie n'existe pas.");
            return;
        }

        string[] subDirs = Directory.GetDirectories(outputFolder);
        List<string> foldersToProcess = subDirs.Length > 0
            ? new List<string>(subDirs)
            : new List<string> { outputFolder };

        List<string> commands = new List<string>();

        foreach (string folder in foldersToProcess)
        {
            string folderName = new DirectoryInfo(folder).Name;

            string profileCLI = profile switch
            {
                "Splat3" => "Splat3",
                "MCMC" => "Splat MCMC",
                "ADC" => "Splat ADC",
                _ => throw new System.Exception("Profil inconnu")
            };

            string outputFile = Path.Combine(outputFolder, $"{folderName}.{outputFormat}");

            string command = $"\"{postshotPath}\" train -i \"{folder}\" -s {trainStep} --profile \"{profileCLI}\"";

            if (outputFormat == "ply")
                command += $" --export-splat-ply \"{outputFile}\"";
            else
                command += $" --output \"{outputFile}\"";

            commands.Add(command);
        }

        string tempBatPath = Path.Combine(Path.GetTempPath(), "postshot_batch.bat");

        File.WriteAllLines(tempBatPath, commands);

        var psi = new System.Diagnostics.ProcessStartInfo
        {
            FileName = "cmd.exe",
            Arguments = $"/K \"{tempBatPath}\"",
            UseShellExecute = true
        };

        System.Diagnostics.Process.Start(psi);
    }





    private void Update()
    {
        if (gizmoViewer == null)
        {

            CameraDomeGizmo[] scripts = FindObjectsOfType<CameraDomeGizmo>();
            if (scripts.Length > 0)
                foreach (CameraDomeGizmo script in scripts)
                {
                    gizmoViewer = script.gameObject;
                }
            else if (gizmoViewerPrefab != null)
                gizmoViewer = Instantiate(gizmoViewerPrefab);


        }
        else
        {
            gizmoViewer.GetComponent<CameraDomeGizmo>().target = target;
            gizmoViewer.GetComponent<CameraDomeGizmo>().numRings = numRings;
            gizmoViewer.GetComponent<CameraDomeGizmo>().viewsPerRing = viewsPerRing;
            gizmoViewer.GetComponent<CameraDomeGizmo>().radius = radius;
            gizmoViewer.GetComponent<CameraDomeGizmo>().height = height;

            gizmoViewer.GetComponent<CameraDomeGizmo>().mode = tabIndex;
            gizmoViewer.GetComponent<CameraDomeGizmo>().volumeCenter = volumeCenter;
            gizmoViewer.GetComponent<CameraDomeGizmo>().volumeSize = volumeSize;
            gizmoViewer.GetComponent<CameraDomeGizmo>().subdivX = subdivX;
            gizmoViewer.GetComponent<CameraDomeGizmo>().subdivY = subdivY;
            gizmoViewer.GetComponent<CameraDomeGizmo>().subdivZ = subdivZ;
            gizmoViewer.GetComponent<CameraDomeGizmo>().showGrid = ShowGrid;

        }
    }



    private void DrawSphericalCaptureUI()
    {
        GUILayout.Label("Capture Settings", EditorStyles.boldLabel);

        target = (Transform)EditorGUILayout.ObjectField("Target", target, typeof(Transform), true);
        numRings = EditorGUILayout.IntField("Number of Rings", numRings);
        viewsPerRing = EditorGUILayout.IntField("Views per Ring", viewsPerRing);
        radius = EditorGUILayout.FloatField("Radius", radius);
        height = EditorGUILayout.FloatField("Height", height);

        GUILayout.Space(10);
        if (!isRunning)
            if (GUILayout.Button("Capture and Export COLMAP"))
            {
                if (cameraToUse == null || target == null || string.IsNullOrEmpty(outputFolder))
                {
                    Debug.LogError("Please assign a camera and an output folder.");
                    return;
                }

                StartCaptureDome(runtimeAnim);
            }
    }
    private Vector3 volumeCenter = Vector3.zero;
    private Vector3 volumeSize = new Vector3(5, 5, 5);
    private int subdivX = 2, subdivY = 2, subdivZ = 2;

    private void DrawVolumeCaptureUI()
    {
        GUILayout.Label("Volume Settings", EditorStyles.boldLabel);
        volumeCenter = EditorGUILayout.Vector3Field("Volume Center", volumeCenter);
        volumeSize = EditorGUILayout.Vector3Field("Volume Size", volumeSize);
        subdivX = EditorGUILayout.IntField("Subdivisions X", subdivX);
        subdivY = EditorGUILayout.IntField("Subdivisions Y", subdivY);
        subdivZ = EditorGUILayout.IntField("Subdivisions Z", subdivZ);
        ShowGrid = EditorGUILayout.Toggle("Show Grid", ShowGrid);
        
        if (!isRunning)
            if (GUILayout.Button("Capture and Export COLMAP"))
            {
                if (cameraToUse == null || string.IsNullOrEmpty(outputFolder))
                {
                    Debug.LogError("Please assign a camera and an output folder.");
                    return;
                }

                StartCaptureVolume(runtimeAnim);
            }
    }

    public IEnumerator CaptureViewsAndExportColmap(string outAdd)
    {
        isRunning = true;

        string folderPath = outputFolder+outAdd;
        Directory.CreateDirectory(folderPath);
        // === cameras.txt ===
        string camerasTxt = Path.Combine(folderPath, "cameras.txt");


        float fov = cameraToUse.fieldOfView;
        float fy = 0.5f * h / Mathf.Tan(0.5f * fov * Mathf.Deg2Rad);
        float fx = fy; 

        float cx = w / 2f;
        float cy = h / 2f;


        using (StreamWriter camWriter = new StreamWriter(camerasTxt))
        {
            camWriter.WriteLine("# Camera list with one line of data per camera:");
            camWriter.WriteLine("#   CAMERA_ID, MODEL, WIDTH, HEIGHT, PARAMS[]");
            camWriter.WriteLine($"1 PINHOLE {w} {h} {fx.ToString(CultureInfo.InvariantCulture)} {fy.ToString(CultureInfo.InvariantCulture)} {cx} {cy}");
        }

        // === images.txt ===
        string imagesTxt = Path.Combine(folderPath, "images.txt");
        using (StreamWriter imgWriter = new StreamWriter(imagesTxt))
        {
            imgWriter.WriteLine("# Image list with two lines per image:");
            imgWriter.WriteLine("# IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, IMAGE_NAME");
            imgWriter.WriteLine("# POINTS2D[] as X, Y, POINT3D_ID");

            // Check if the project is in Linear space. If yes, force sRGB conversion. Otherwise, leave it as Default.
            RenderTextureReadWrite rwMode = PlayerSettings.colorSpace == ColorSpace.Linear ? RenderTextureReadWrite.sRGB : RenderTextureReadWrite.Default;
            RenderTexture rt = new RenderTexture(w, h, 32, RenderTextureFormat.ARGB32, rwMode);
            Texture2D tex = new Texture2D(w, h, TextureFormat.RGBA32, false);

            int imageId = 1;
            int batchSize = 40;
            int batchCounter = 0;
            int totalImages = viewsPerRing*numRings;
            int currentImage = 0;
            StreamWriter writer3D = new StreamWriter(Path.Combine(folderPath, "points3D.txt"));
            writer3D.WriteLine("# 3D point list with one line of data per point:");
            writer3D.WriteLine("# POINT3D_ID, X, Y, Z, R, G, B, ERROR, TRACK[] as (IMAGE_ID, POINT2D_IDX)");
            int pointId = 1;
            
            foreach (MeshRenderer r in GameObject.FindObjectsOfType<MeshRenderer>())
            {
                if (!r.GetComponent<Collider>())
                {
                    r.gameObject.AddComponent<MeshCollider>();
                }
            }

            foreach (SkinnedMeshRenderer r in GameObject.FindObjectsOfType<SkinnedMeshRenderer>())
            {
                if (!r.GetComponent<MeshCollider>())
                {
                    r.gameObject.AddComponent<MeshCollider>();
                }

                Mesh bakedMesh = new Mesh();
                r.BakeMesh(bakedMesh);

                r.GetComponent<MeshCollider>().sharedMesh = null; 
                r.GetComponent<MeshCollider>().sharedMesh = bakedMesh;


            }

            for (int ring = 0; ring < numRings; ring++)
            {
                float elevation = Mathf.Lerp(-Mathf.PI / 4, Mathf.PI / 4, (float)ring / (numRings - 1)); 

                for (int i = 0; i < viewsPerRing; i++)
                {
                    float progress = (float)currentImage / totalImages;
                    EditorUtility.DisplayProgressBar("Capture Views", $"Image {currentImage + 1} / {totalImages} ", progress);
                    float azimuth = i * Mathf.PI * 2 / viewsPerRing;

                    float x = radius * Mathf.Cos(elevation) * Mathf.Cos(azimuth);
                    float y = radius * Mathf.Sin(elevation);
                    float z = radius * Mathf.Cos(elevation) * Mathf.Sin(azimuth);

                    Vector3 position = target.position + new Vector3(x, y + height, z);
                    cameraToUse.transform.position = position;
                    cameraToUse.transform.LookAt(target);

                    GetColmapPoseFromUnityCamera(cameraToUse, out Quaternion q, out Vector3 t);

                    string imageName = $"view_{imageId:D3}.png";
                    string imagePath = Path.Combine(folderPath, imageName);

                    cameraToUse.clearFlags = CameraClearFlags.SolidColor;
                    cameraToUse.backgroundColor = new Color(0, 0, 0, 0);

                    cameraToUse.targetTexture = rt;
                    cameraToUse.Render();
                    RenderTexture.active = rt;
                    tex.ReadPixels(new Rect(0, 0, w, h), 0, 0);
                    tex.Apply();
                    CapturePointCloudFromCamera(cameraToUse, tex, rays, writer3D, ref pointId);

                    File.WriteAllBytes(imagePath, tex.EncodeToPNG());

                    imgWriter.WriteLine($"{imageId} {q.w.ToString(CultureInfo.InvariantCulture)} {q.x.ToString(CultureInfo.InvariantCulture)} {q.y.ToString(CultureInfo.InvariantCulture)} {q.z.ToString(CultureInfo.InvariantCulture)} {t.x.ToString(CultureInfo.InvariantCulture)} {t.y.ToString(CultureInfo.InvariantCulture)} {t.z.ToString(CultureInfo.InvariantCulture)} 1 {imageName}");
                    imgWriter.WriteLine();

                    imageId++;

                    batchCounter++;
                    currentImage++;
                    if (cancel)
                    {
                        Debug.LogWarning("Capture canceled.");
                        EditorUtility.ClearProgressBar();
                        cancel = false;
                        isRunning = false;

                        yield break;
                    }

                    if (batchCounter >= batchSize)
                    {
                        batchCounter = 0;

                        cameraToUse.targetTexture = null;
                        RenderTexture.active = null;
                        GL.Clear(true, true, Color.clear);

                        tex = null;
                        rt.Release();
                        rt = null;

                        DestroyImmediate(rt);
                        DestroyImmediate(tex);

                        EditorUtility.UnloadUnusedAssetsImmediate();
                        AssetDatabase.SaveAssets();
                        EditorApplication.QueuePlayerLoopUpdate();

                        GC.Collect();
                        GC.WaitForPendingFinalizers();



                        rt = new RenderTexture(w, h, 32, RenderTextureFormat.ARGB32, rwMode);
                        tex = new Texture2D(w, h, TextureFormat.RGBA32, false);

                        yield return null;

                    }

                }
            }
            writer3D.Close();

            cameraToUse.targetTexture = null;
            RenderTexture.active = null;
            DestroyImmediate(rt);
            DestroyImmediate(tex);
        }


        Debug.Log("Captures + COLMAP files finished !");
        AssetDatabase.Refresh();
        EditorUtility.ClearProgressBar();

        if (!runtimeAnim)
            EditorUtility.RevealInFinder(folderPath);
        isRunning = false;

        if (!runtimeAnim && TrainPostShot)
        {
            RunPostshotBatch();
        }

        yield return new WaitForEndOfFrame();
        if (EditorApplication.isPaused == true)
            EditorApplication.isPaused = false;

    }
    private IEnumerator WaitForPlayAndCapture(bool isDome)
    {
        if (!EditorApplication.isPlaying)
        {
            Debug.LogError("You need to be in Play Mode to record a sequence.");
        }
        else
        {
            yield return new EditorWaitForSeconds(0.5f);

            int totalFrames = Mathf.RoundToInt(duration * fbs);


            GameObject.FindObjectsOfType<CameraDomeGizmo>()[0].GetComponent<CameraDomeGizmo>().currentTime = 0;
            isRunning = true;
            for (int i = 0; i < totalFrames; i++)
            {
                Debug.Log($"[Editor] Step to frame {i}");


                float targetTime = i / (float)fbs;
                while (GameObject.FindObjectsOfType<CameraDomeGizmo>()[0].GetComponent<CameraDomeGizmo>().currentTime < targetTime)
                {
                    yield return null;
                }

                EditorApplication.isPaused = true;

                if (cancel)
                {
                    Debug.LogWarning("Capture canceled.");
                    EditorUtility.ClearProgressBar();
                    cancel = false;
                    isRunning = false;

                    yield break;
                }
                var window = GetWindow<CameraCaptureEditor>();

                yield return window.StartCoroutine(
                    isDome ? window.CaptureViewsAndExportColmap("/" + i + "/")
                           : window.CaptureVolumeViewsAndExportColmap("/" + i + "/")
                );
            }
            EditorUtility.RevealInFinder(outputFolder);


            if (TrainPostShot)
                RunPostshotBatch();
            EditorApplication.isPlaying = false;
        }
    }


    public IEnumerator CaptureVolumeViewsAndExportColmap(string outAdd)
    {


        isRunning = true;
        string folderPath = outputFolder + outAdd;
        Directory.CreateDirectory(folderPath);

        // === cameras.txt ===
        string camerasTxt = Path.Combine(folderPath, "cameras.txt");


        float fov = cameraToUse.fieldOfView;
        float fy = 0.5f * h / Mathf.Tan(0.5f * fov * Mathf.Deg2Rad);
        float fx = fy;

        float cx = w / 2f;
        float cy = h / 2f;


        using (StreamWriter camWriter = new StreamWriter(camerasTxt))
        {
            camWriter.WriteLine("# Camera list with one line of data per camera:");
            camWriter.WriteLine("#   CAMERA_ID, MODEL, WIDTH, HEIGHT, PARAMS[]");
            camWriter.WriteLine($"1 PINHOLE {w} {h} {fx.ToString(CultureInfo.InvariantCulture)} {fy.ToString(CultureInfo.InvariantCulture)} {cx} {cy}");
        }

        // === images.txt ===
        string imagesTxt = Path.Combine(folderPath, "images.txt");
        using (StreamWriter imgWriter = new StreamWriter(imagesTxt))
        {
            imgWriter.WriteLine("# Image list with two lines per image:");
            imgWriter.WriteLine("# IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, IMAGE_NAME");
            imgWriter.WriteLine("# POINTS2D[] as X, Y, POINT3D_ID");

            // Check if the project is in Linear space. If yes, force sRGB conversion. Otherwise, leave it as Default.
            RenderTextureReadWrite rwMode = PlayerSettings.colorSpace == ColorSpace.Linear ? RenderTextureReadWrite.sRGB : RenderTextureReadWrite.Default;
            RenderTexture rt = new RenderTexture(w, h, 32, RenderTextureFormat.ARGB32, rwMode);

            Texture2D tex = new Texture2D(w, h, TextureFormat.RGBA32, false);


            int imageId = 1;
            Vector3 step = new Vector3(volumeSize.x / subdivX, volumeSize.y / subdivY, volumeSize.z / subdivZ);
            int totalCells = subdivX * subdivY * subdivZ;
            List<Vector3> directions = GenerateCustomSphericalDirections();

            int totalImages = subdivX * subdivY * subdivZ * directions.Count;
            int currentImage = 0;
            int batchSize = 40;
            int batchCounter = 0;
            int imagesSkipped = 0;

            StreamWriter writer3D = new StreamWriter(Path.Combine(folderPath, "points3D.txt"));
            writer3D.WriteLine("# 3D point list with one line of data per point:");
            writer3D.WriteLine("# POINT3D_ID, X, Y, Z, R, G, B, ERROR, TRACK[] as (IMAGE_ID, POINT2D_IDX)");
            int pointId = 1;


            for (int x = 0; x < subdivX; x++)
            {
                for (int y = 0; y < subdivY; y++)
                {
                    for (int z = 0; z < subdivZ; z++)
                    {
                        Vector3 cellCenter = volumeCenter - volumeSize / 2f + step * 0.5f + new Vector3(x * step.x, y * step.y, z * step.z);



                        foreach (Vector3 dir in directions)
                        {


                            float progress = (float)currentImage / totalImages;
                            EditorUtility.DisplayProgressBar("Capture 3D Volume", $"Image {currentImage + 1} / {totalImages} | Images Skipped : {imagesSkipped}", progress);

                            cameraToUse.transform.position = cellCenter;
                            cameraToUse.transform.rotation = Quaternion.LookRotation(dir);

                            GetColmapPoseFromUnityCamera(cameraToUse, out Quaternion q, out Vector3 t);

                            string imageName = $"vol_{imageId:D4}.png";
                            string imagePath = Path.Combine(folderPath, imageName);


                            Plane[] planes = GeometryUtility.CalculateFrustumPlanes(cameraToUse);
                            bool objectVisible = false;

                            foreach (MeshRenderer r in GameObject.FindObjectsOfType<MeshRenderer>())
                            {
                                if (!r.GetComponent<Collider>())
                                {
                                    r.gameObject.AddComponent<MeshCollider>();
                                }
                            }

                            foreach (SkinnedMeshRenderer r in GameObject.FindObjectsOfType<SkinnedMeshRenderer>())
                            {
                                if (!r.GetComponent<MeshCollider>())
                                {
                                    r.gameObject.AddComponent<MeshCollider>();
                                }

                                Mesh bakedMesh = new Mesh();
                                r.BakeMesh(bakedMesh);

                                r.GetComponent<MeshCollider>().sharedMesh = null;
                                r.GetComponent<MeshCollider>().sharedMesh = bakedMesh;


                            }

                            foreach (Renderer renderer in GameObject.FindObjectsOfType<Renderer>())
                            {
                                if (GeometryUtility.TestPlanesAABB(planes, renderer.bounds))
                                {
                                    objectVisible = true;
                                    break;
                                }
                            }

                            if (!objectVisible)
                            {
                                imagesSkipped++;
                                continue;
                            }


                            cameraToUse.clearFlags = CameraClearFlags.SolidColor;
                            cameraToUse.backgroundColor = new Color(0, 0, 0, 0); 

                            cameraToUse.targetTexture = rt;
                            cameraToUse.Render();
                            RenderTexture.active = rt;
                            tex.ReadPixels(new Rect(0, 0, w, h), 0, 0);
                            tex.Apply();
                            CapturePointCloudFromCamera(cameraToUse, tex, rays, writer3D, ref pointId);

                            byte[] pngData = tex.EncodeToPNG();
                            File.WriteAllBytes(imagePath, pngData);
                            pngData = null;


                            imgWriter.WriteLine($"{imageId} {q.w.ToString(CultureInfo.InvariantCulture)} {q.x.ToString(CultureInfo.InvariantCulture)} {q.y.ToString(CultureInfo.InvariantCulture)} {q.z.ToString(CultureInfo.InvariantCulture)} {t.x.ToString(CultureInfo.InvariantCulture)} {t.y.ToString(CultureInfo.InvariantCulture)} {t.z.ToString(CultureInfo.InvariantCulture)} 1 {imageName}");
                            imgWriter.WriteLine();

                            imageId++;

                            currentImage++;

                            batchCounter++;

                            if (cancel)
                            {
                                Debug.LogWarning("Capture canceled.");
                                EditorUtility.ClearProgressBar();
                                cancel = false;
                                isRunning = false;

                                yield break;
                            }

                            if (batchCounter >= batchSize)
                            {
                                batchCounter = 0;

                                cameraToUse.targetTexture = null;
                                RenderTexture.active = null;
                                GL.Clear(true, true, Color.clear);

                                tex = null;
                                rt.Release();
                                rt = null;

                                DestroyImmediate(rt);
                                DestroyImmediate(tex);

                                EditorUtility.UnloadUnusedAssetsImmediate();
                                AssetDatabase.SaveAssets();
                                EditorApplication.QueuePlayerLoopUpdate();

                                GC.Collect();
                                GC.WaitForPendingFinalizers();



                                rt = new RenderTexture(w, h, 32, RenderTextureFormat.ARGB32, rwMode);
                                tex = new Texture2D(w, h, TextureFormat.RGBA32, false);

                                yield return null;

                            }



                        }
                    }
                }
            }

            writer3D.Close();

            cameraToUse.targetTexture = null;
            RenderTexture.active = null;

            DestroyImmediate(rt);
            DestroyImmediate(tex);


        }


        Debug.Log("Captures + COLMAP files finished !");
        AssetDatabase.Refresh();
        EditorUtility.ClearProgressBar();

        if (!runtimeAnim)
            EditorUtility.RevealInFinder(folderPath);
        isRunning = false;

        if (!runtimeAnim && TrainPostShot)
        {
            RunPostshotBatch();
        }

        yield return new WaitForEndOfFrame();
        if (EditorApplication.isPaused == true)
            EditorApplication.isPaused = false;

    }
    private static void StartCaptureVolume(bool isRuntime)
    {
        var window = GetWindow<CameraCaptureEditor>();
        if (isRuntime)
        {

            window.captureCoroutine = EditorCoroutineUtility.StartCoroutine(window.WaitForPlayAndCapture(false), window);
        }
        else
            window.captureCoroutine = EditorCoroutineUtility.StartCoroutine(window.CaptureVolumeViewsAndExportColmap(""), window);
    }


    private static void StartCaptureDome(bool isRuntime)
    {
        var window = GetWindow<CameraCaptureEditor>();
        if (isRuntime)
        {

            window.captureCoroutine = EditorCoroutineUtility.StartCoroutine(window.WaitForPlayAndCapture(true), window);
        }
        else
            window.captureCoroutine = EditorCoroutineUtility.StartCoroutine(window.CaptureViewsAndExportColmap(""), window);
    }


    private List<Vector3> GenerateCustomSphericalDirections()
    {
        List<Vector3> directions = new List<Vector3>();

        for (int i = 0; i < 8; i++)
        {
            float azimuth = i * 45f;
            Quaternion rot = Quaternion.Euler(0f, azimuth, 0f);
            directions.Add(rot * Vector3.forward);
        }

        for (int i = 0; i < 4; i++)
        {
            float azimuth = i * 90f;
            Quaternion rot = Quaternion.Euler(45f, azimuth, 0f);
            directions.Add(rot * Vector3.forward);
        }

        for (int i = 0; i < 4; i++)
        {
            float azimuth = i * 90f;
            Quaternion rot = Quaternion.Euler(-45f, azimuth, 0f);
            directions.Add(rot * Vector3.forward);
        }

        directions.Add(Vector3.up);

        directions.Add(Vector3.down);

        return directions;
    }
    void CapturePointCloudFromCamera(Camera cam, Texture2D tex, int rayCount, StreamWriter writer, ref int pointId)
    {
        int width = tex.width;
        int height = tex.height;

        int sqrtRayCount = Mathf.CeilToInt(Mathf.Sqrt(rayCount));
        float stepX = width / (float)sqrtRayCount;
        float stepY = height / (float)sqrtRayCount;

        int noCloudLayer = LayerMask.NameToLayer("NoCloud");
        int layerMask = ~(1 << noCloudLayer); 

        for (int i = 0; i < sqrtRayCount; i++)
        {
            for (int j = 0; j < sqrtRayCount; j++)
            {
                float px = i * stepX + stepX / 2f;
                float py = j * stepY + stepY / 2f;

                Ray ray = cam.ScreenPointToRay(new Vector3(px, py, 0));

                // ADDED: QueryTriggerInteraction.Ignore so we don't hit invisible trigger boxes
                if (Physics.Raycast(ray, out RaycastHit hit, Mathf.Infinity, layerMask, QueryTriggerInteraction.Ignore))
                {
                    Color color = tex.GetPixel((int)px, (int)py);

                    // ADDED: If the pixel is fully transparent (Background/Skybox), skip this point!
                    // This prevents invisible colliders from generating flat walls/ground.
                    if (color.a < 0.1f)
                        continue;

                    Vector3 worldPos = UnityToColmapPoint(hit.point);

                    int r = Mathf.Clamp((int)(color.r * 255), 0, 255);
                    int g = Mathf.Clamp((int)(color.g * 255), 0, 255);
                    int b = Mathf.Clamp((int)(color.b * 255), 0, 255);

                    writer.WriteLine($"{pointId} {worldPos.x.ToString(CultureInfo.InvariantCulture)} {worldPos.y.ToString(CultureInfo.InvariantCulture)} {worldPos.z.ToString(CultureInfo.InvariantCulture)} {r} {g} {b} 1.0");
                    pointId++;
                }
            }
        }
    }
}