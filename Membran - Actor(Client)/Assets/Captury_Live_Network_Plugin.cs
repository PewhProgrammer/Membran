using UnityEngine;
using UnityEngine.UI;
using System;
using System.Text;
using System.Collections;
using System.Collections.Generic;
using System.Threading;
using System.Runtime.InteropServices;

//=================================
// define captury class structures
//=================================
[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct CapturyJoint
{
    [MarshalAs(UnmanagedType.ByValArray, SizeConst = 24)]
    public byte[] name;
    public int parent;
    public float ox, oy, oz;
    public float rx, ry, rz;
}

[StructLayout(LayoutKind.Sequential, Pack = 1)]
public struct CapturyActor
{
    [MarshalAs(UnmanagedType.ByValArray, SizeConst = 32)]
    public byte[] name;
    public int id;
    public int numJoints;
    public IntPtr joints;
    public int numBlobs;
    public int padding2;
    public IntPtr blobs;
    public int padding;
    public int padding3;
}

[StructLayout(LayoutKind.Sequential)]
public struct CapturyPose
{
    public int actor;
    public long timestamp;
    public int numValues;
    public IntPtr values;
}

[StructLayout(LayoutKind.Sequential)]
public struct CapturyImage
{
    public int width;
    public int height;
    public int camera;
    public ulong timestamp;
    public IntPtr data;
}

[StructLayout(LayoutKind.Sequential)]
public struct CapturyTransform
{
    public float rx;  // rotation euler angles
    public float ry;
    public float rz;
    public float tx; // translation
    public float ty;
    public float tz;
}



//==========================================
// internal structures that are more useful
//==========================================
[Serializable]
public class CapturySkeletonJoint
{
    public string name;
    public int parent;
    public Vector3 offset;
    public Vector3 orientation;
    public Transform transform;
}

[Serializable]
public class CapturySkeleton
{
    public string name;
    public int id;
    public Transform reference;
    public CapturySkeletonJoint[] joints;
}

[Serializable]
public class CapturyMarkerTransform
{
    public Quaternion   rotation;
    public Vector3      translation;
    public UInt64       timestamp;
    public float        bestAccuracy;
    public bool         consumed;
}

//====================
// the network plugin
//====================
public class Captury_Live_Network_Plugin : Photon.PunBehaviour
{
    //=============================================
    // import the functions from RemoteCaptury dll
    //=============================================
    [DllImport("RemoteCaptury")]
    private static extern int Captury_connect(string ip, ushort port);
    [DllImport("RemoteCaptury")]
    private static extern int Captury_disconnect();
    [DllImport("RemoteCaptury")]
    private static extern int Captury_getActors(out IntPtr actorData);
    [DllImport("RemoteCaptury")]
    private static extern int Captury_startStreaming(int what);
    [DllImport("RemoteCaptury")]
    private static extern int Captury_stopStreaming();
    [DllImport("RemoteCaptury")]
    private static extern IntPtr Captury_getCurrentPose(IntPtr actor);
    [DllImport("RemoteCaptury")]
    private static extern void Captury_freePose(IntPtr pose);
    [DllImport("RemoteCaptury")]
    private static extern void Captury_requestTexture(IntPtr actor);
    [DllImport("RemoteCaptury")]
    private static extern IntPtr Captury_getTexture(IntPtr actor);
    [DllImport("RemoteCaptury")]
    private static extern void Captury_freeImage(IntPtr image);
    [DllImport("RemoteCaptury")]
    private static extern int Captury_setRotationConstraint(int actorId, int jointIndex, IntPtr rotation, UInt64 timestamp, float weight);
    [DllImport("RemoteCaptury")]
    private static extern UInt64 Captury_getMarkerTransform(IntPtr actor, int jointIndex, IntPtr transform);
    [DllImport("RemoteCaptury")]
    private static extern UInt64 Captury_synchronizeTime();
    [DllImport("RemoteCaptury")]
    private static extern UInt64 Captury_getTime();
    [DllImport("RemoteCaptury")]
    private static extern Int64 Captury_getTimeOffset();
    [DllImport("RemoteCaptury")]
    private static extern IntPtr Captury_getLastErrorMessage();
    [DllImport("RemoteCaptury")]
    private static extern void Captury_freeErrorMessage(IntPtr msg);

    public string host = "127.0.0.1";
    public ushort port = 2101;
    public float scaleFactor = 0.001f; // mm to m
    public Transform cameraTransform;
    public int playerActorIndex = 10;
    private int playerActor = -1;
    public int actorCheckTimeout = 500; // in ms
    private int actorNumJoints = 29;

    public GameObject actorTemplateObject; // this object is cloned for each actor
    public GameObject meshNode; 
    private GameObject gameActor;
    private int actorID = 1;

    private string headJointName = "Head";
    public Transform headTransform;

    // threading data for communication with server
    private Thread communicationThread;
    private Mutex communicationMutex = new Mutex();
    private bool communicationFinished = false;

    // internal variables
    private bool isConnected = false;
    private bool isSetup = false;

    // skeleton data from Captury
    private Dictionary<int, IntPtr> actorPointers = new Dictionary<int, IntPtr>();
    private Dictionary<int, CapturyActor> actors = new Dictionary<int, CapturyActor>();
    private Dictionary<int, int> actorFound = new Dictionary<int, int>();
    private Dictionary<int, CapturySkeleton> skeletons = new Dictionary<int, CapturySkeleton>();
    private CapturySkeleton skeleton; 
    private Dictionary<int, CapturyMarkerTransform> headTransforms = new Dictionary<int, CapturyMarkerTransform>();
	
	// graphical representation of actors
	private Dictionary<int, UnityEngine.GameObject> actorObjects = new Dictionary<int, UnityEngine.GameObject>();

    // texture access data
    private int textureRequestId = -1;
    public SkinnedMeshRenderer meshRenderer;
    private Fading fadeScript;

    void OnGUI()
    {
        GUI.Label(new Rect(20, 80, 1000, 20), PhotonNetwork.connectionStateDetailed.ToString());
        GUI.Label(new Rect(20, 20, 1000, 20), "Players in room: " + PhotonNetwork.playerList.Length);
    }

    override
    public void OnConnectedToMaster()
    {
        RoomOptions rOpts = new RoomOptions()
        {
            IsVisible = false,
            MaxPlayers = 2
        };

        PhotonNetwork.JoinOrCreateRoom("Actor", rOpts, TypedLobby.Default);
    }

    override
    public void OnJoinedRoom()
    {
        buildSkeleton();
    }

    void buildSkeleton()
    {
        //gameActor = (GameObject)Instantiate(actorTemplateObject);
        //gameActor.SetActive(true);

        //meshNode.AddComponent<Fading>().enabled = true;

        
        skeleton = new CapturySkeleton();
        ConvertActor(ref skeleton);
        //ConnectSkeleton(gameActor.transform);
        ConnectSkeleton(actorTemplateObject.transform);

     

        Debug.Log("built main actor");
    }


    [PunRPC]
    public void sendActorSkeleton(CapturyActor actor,CapturySkeleton skeleton)
    {
        Debug.Log("received actor " + actor.name + " and skeleton " + skeleton.name);
    }


        //=============================
        // this is run once at startup
        //=============================
        void Start()
    {
        Debug.Log("Conneting to PhotonServer...");
        PhotonNetwork.ConnectUsingSettings("v1.0");
        // start the connection thread
        //communicationThread = new Thread(lookForActors);
        //communicationThread.Start();
    }


    //==========================
    // this is run once at exit
    //==========================
    void OnDisable()
    {
        communicationFinished = true;
        //communicationThread.Join();
    }


    //============================
    // this is run once per frame
    //============================
    void Update()
    {
        if (Input.GetKeyDown(KeyCode.Escape))
            Application.Quit();

        // only perform if we are actually connected
        if (isConnected)
        {
            // check for keypresses to request texture
            if (Input.GetKeyUp(KeyCode.T) && playerActor != -1)
                RequestTexture(playerActor);

            // make sure we lock access before doing anything
            communicationMutex.WaitOne();
			
            // remove actor objects for actors that have disappeared
			List<int> actorsToRemove = new List<int>();
			foreach (KeyValuePair<int, GameObject> kvp in actorObjects)
            {
				if (actors.ContainsKey(kvp.Key))
					continue;
				
				actorsToRemove.Add(kvp.Key);
			}
			foreach (int id in actorsToRemove)
			{
				Destroy(actorObjects[id]);
				actorObjects.Remove(id);
			}

            // fetch current pose for all skeletons
			int i = -1;
            foreach (KeyValuePair<int, CapturyActor> kvp in actors)
            {
				++i;
                // get the actor id
                int actorID = kvp.Key;

				// add game object for actor
				if (!actorObjects.ContainsKey(actorID)) {
					GameObject actor = (GameObject) Instantiate(actorTemplateObject);
					actor.SetActive(true);
					actorObjects.Add(actorID, actor);
                    ConnectSkeleton(actor.transform);
				}

                // check if we need to initialize the player
                if (playerActor == -1 && i == playerActorIndex)
                {
                    playerActor = actorID;
                    Captury_synchronizeTime();
                }

                // check if the actor is mapped to something, if not, ignore
                if (skeletons[actorID].reference == null)
                    continue;
                // get pointer to pose
                IntPtr poseData = Captury_getCurrentPose(actorPointers[actorID]);

                // check if we actually got data, if not, continue
                if (poseData == IntPtr.Zero)
                {
                    // something went wrong, get error message
                    IntPtr msg = Captury_getLastErrorMessage();
                    string errmsg = Marshal.PtrToStringAnsi(msg);
                    Debug.Log("Stream error: " + errmsg);
                    Captury_freeErrorMessage(msg);
                    continue;
                }

 //               Debug.Log("received pose for " + actorID);

                // convert the pose
                CapturyPose pose = (CapturyPose)Marshal.PtrToStructure(poseData, typeof(CapturyPose));

                // get the data into a float array
                float[] values = new float[pose.numValues];
                Marshal.Copy(pose.values, values, 0, pose.numValues);

                this.photonView.RPC("sendValues", PhotonTargets.Others, values,actorID);

                // now loop over all joints
                Vector3 pos = new Vector3();
                Vector3 rot = new Vector3();
                for (int jointID = 0; jointID < skeletons[actorID].joints.Length; jointID++)
                {
                    // ignore any joints that do not map to a transform
                    if (skeletons[actorID].joints[jointID].transform == null)
                        continue;

                    // set offset and rotation
                    int baseIndex = jointID * 6;
                    pos.Set(values[baseIndex + 0], values[baseIndex + 1], values[baseIndex + 2]);
                    rot.Set(values[baseIndex + 3], values[baseIndex + 4], values[baseIndex + 5]);

                    skeletons[actorID].joints[jointID].transform.position = ConvertPosition(pos);
                    skeletons[actorID].joints[jointID].transform.rotation = ConvertRotation(rot);
//				    if (jointID == 0)
//					Debug.Log("updating joint" + skeletons[actorID].joints[jointID].name + " " +  pos);
                }

                // finally, free the pose data again, as we are finished
                Captury_freePose(poseData);
            }

            communicationMutex.ReleaseMutex();
        }
    }


    [PunRPC]
    public void sendValues(float[] values) //receive values
    {
        //Debug.Log("received Values with Length: " + values.Length);
        
        // now loop over all joints
        Vector3 pos = new Vector3();
        Vector3 rot = new Vector3();
        for (int jointID = 0; jointID < skeleton.joints.Length; jointID++)
        {
            // ignore any joints that do not map to a transform
            if (skeleton.joints[jointID].transform == null)
                continue;
            // set offset and rotation
            int baseIndex = jointID * 6;
            pos.Set(values[baseIndex + 0], values[baseIndex + 1], values[baseIndex + 2]);
            rot.Set(values[baseIndex + 3], values[baseIndex + 4], values[baseIndex + 5]);

            skeleton.joints[jointID].transform.position = ConvertPosition(pos);
            skeleton.joints[jointID].transform.rotation = ConvertRotation(rot);
            //				    if (jointID == 0)
            //					Debug.Log("updating joint" + skeletons[actorID].joints[jointID].name + " " +  pos);
        }
        
    }


    //=====================================================================
    // helper functions to update the texture of an actor over the network
    //=====================================================================
    private void RequestTexture(int id)
    {
        // if the actor doesnt exist, ignore
        if (textureRequestId != -1 || !actors.ContainsKey(id))
            return;

        textureRequestId = id;

        Debug.Log("Requesting texture...");
        // request new texture for the actor
        Captury_requestTexture(actorPointers[textureRequestId]);
        // wait for a few seconds for the result, to make sure the texture is really ready
        StartCoroutine("TextureEvent");
    }

    private IEnumerator TextureEvent()
    {
        // ignore if we have invalid texture id
        if (textureRequestId != -1)
        {
            // wait for 3 seconds
            yield return new WaitForSeconds(3.0f);

            // now actually grab the data
            Debug.Log("Getting texture...");
            // now check for results
            IntPtr textureData = Captury_getTexture(actorPointers[textureRequestId]);

            // check if we really got something
            if (textureData != IntPtr.Zero)
            {
                CapturyImage image = new CapturyImage();
                image = (CapturyImage)Marshal.PtrToStructure(textureData, typeof(CapturyImage));

                Debug.Log("Got image " + image.width + "x" + image.height);
                // check for received data
                if (image.width != 0 && image.height != 0)
                {
                    // create texture
                    Texture2D tex = new Texture2D(image.width, image.height);
                    byte[] data = new byte[image.width * image.height * 3];

                    // grab the actual data
                    Marshal.Copy(image.data, data, 0, image.width * image.height * 3);

                    // copy data from data to cols
                    for (int y = 0; y < image.height; y++)
                    {
                        for (int x = 0; x < image.width; x++)
                        {
                            int index = (y * image.width + x) * 3;
                            tex.SetPixel(x, y, new Color32(data[index + 0], data[index + 1], data[index + 2], 255));
                        }
                    }
                    tex.Apply(true);

                    // now set the texture to the model
                    if (meshRenderer != null)
                    {
                        meshRenderer.material.SetTexture("_MainTex", tex);
                    }
                }

                // free the image in the end
                Captury_freeImage(textureData);
            }

            textureRequestId = -1;
        }
    }

    //===============================================
    // helper function to map an actor to a skeleton
    //===============================================
    private void ConvertActor(ref CapturySkeleton skel)
    {
        if (skel == null)
        {
            Debug.Log("Null skeleton reference");
            return;
        }

        // copy data over
        skel.name = "membran Actor";
        skel.id = actorID;

        // create joints
        int szStruct = Marshal.SizeOf(typeof(CapturyJoint));
        skel.joints = new CapturySkeletonJoint[actorNumJoints];
        for (uint i = 0; i < actorNumJoints; i++)
        {
            // marshall the joints into a new joint struct
            CapturyJoint joint = new CapturyJoint();
            //joint = (CapturyJoint)Marshal.PtrToStructure(new IntPtr(sizeof(int) + (szStruct * i)), typeof(CapturyJoint));

            skel.joints[i] = new CapturySkeletonJoint();
            //skel.joints[i].name = "Hips";//System.Text.Encoding.ASCII.GetString(joint.name);

        }
        skel.joints[0].name = "Hips";
        skel.joints[1].name = "Spine";
        skel.joints[2].name = "Spine1";
        skel.joints[3].name = "Spine2";
        skel.joints[4].name = "Spine3";
        skel.joints[5].name = "Spine4";
        skel.joints[6].name = "Neck";
        skel.joints[7].name = "Head";

        skel.joints[8].name = "HeadEE";
        skel.joints[9].name = "LeftShoulder";
        skel.joints[10].name = "LeftArm";
        skel.joints[11].name = "LeftForeArm";
        skel.joints[12].name = "LeftHand";
        skel.joints[13].name = "LeftHandEE";
        skel.joints[14].name = "RightShoulder";
        skel.joints[15].name = "RightArm";

        skel.joints[16].name = "RightForeArm";
        skel.joints[17].name = "RightHand";
        skel.joints[18].name = "RightHandEE";
        skel.joints[19].name = "LeftUpLeg";
        skel.joints[20].name = "LeftLeg";
        skel.joints[21].name = "LeftFoot";
        skel.joints[22].name = "LeftToeBase";
        skel.joints[23].name = "LeftFootEE";

        skel.joints[24].name = "RightUpLeg";
        skel.joints[25].name = "RightLeg";
        skel.joints[26].name = "RightFoot";
        skel.joints[27].name = "RightToeBase";
        skel.joints[28].name = "RightFootEE";

    }


    //=========================================
    // connect a given transform to a skeleton
    //=========================================
    private void ConnectSkeleton(Transform transform)
    {
        if (transform == null)
            return;
			
        communicationMutex.WaitOne();
        CapturySkeleton skel = skeleton;
        skel.reference = transform;

        for (uint i = 0; i < actorNumJoints; i++)
        {
            // check if the joint name matches a reference transform and assign it
            ArrayList children = skel.reference.transform.GetAllChildren();
            foreach (Transform tra in children)
            {
                if (tra.name.EndsWith(skel.joints[i].name))
                {
                    skel.joints[i].transform = tra;
                    continue;
                }
            }
        }
        communicationMutex.ReleaseMutex();
    }


    //========================================================================================================
    // Helper function to convert a position from a right-handed to left-handed coordinate system (both Y-up)
    //========================================================================================================
    private Vector3 ConvertPosition(Vector3 position)
    {
        position.x *= -scaleFactor;
        position.y *= scaleFactor;
        position.z *= scaleFactor;
        return position;
    }


    //========================================================================================================
    // Helper function to convert a rotation from a right-handed to left-handed coordinate system (both Y-up)
    //========================================================================================================
    private Quaternion ConvertRotation(Vector3 rotation)
    {
        Quaternion qx = Quaternion.AngleAxis(rotation.x, Vector3.right);
        Quaternion qy = Quaternion.AngleAxis(rotation.y, Vector3.down);
        Quaternion qz = Quaternion.AngleAxis(rotation.z, Vector3.back);
        Quaternion qq = qz * qy * qx;
        return qq;
    }

    //===========================================================================================================
    // Helper function to convert a rotation from Unity back to Captury Live (left-handed to right-handed, Y-up)
    //===========================================================================================================
    private Quaternion ConvertRotationToLive(Quaternion rotation)
    {
        Vector3 angles = rotation.eulerAngles;

        Quaternion qx = Quaternion.AngleAxis(angles.x, Vector3.right);
        Quaternion qy = Quaternion.AngleAxis(angles.y, Vector3.down);
        Quaternion qz = Quaternion.AngleAxis(angles.z, Vector3.back);
        Quaternion qq = qz * qy * qx;
        return qq;
    }

}



//==========================================================================
// Helper extension function to get all children from a specified transform
//==========================================================================
public static class TransformExtension
{
    public static ArrayList GetAllChildren(this Transform transform)
    {
        ArrayList children = new ArrayList();
        foreach (Transform child in transform)
        {
            children.Add(child);
            children.AddRange(GetAllChildren(child));
        }
        return children;
    }
}