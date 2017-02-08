    
﻿using UnityEngine;
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
	
	public GameObject actorTemplateObject; // this object is cloned for each actor

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
    private Dictionary<int, CapturyMarkerTransform> headTransforms = new Dictionary<int, CapturyMarkerTransform>();
	
	// graphical representation of actors
	private Dictionary<int, UnityEngine.GameObject> actorObjects = new Dictionary<int, UnityEngine.GameObject>();

    // texture access data
    private int textureRequestId = -1;
    public SkinnedMeshRenderer meshRenderer;

﻿

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

 public void OnJoinedRoom()
 {
     Debug.Log( "OnJoinedRoom() : You Have Joined a Room : " + PhotonNetwork.room.name );
     communicationThread.Start();
 }

    //================================================
    // This function continously looks for new actors
    // It runs in a separate thread
    //================================================
    void lookForActors()
    {
		Debug.Log("looking for actors");
        while (!communicationFinished)
        {
            // wait for actorCheckTimeout ms before continuing
            //			Debug.Log ("Going to sleep...");
            Thread.Sleep(actorCheckTimeout);
            //			Debug.Log ("Waking up...");

            // now look for new data

            // try to connect to captury live
            if (!isSetup)
            {
				Debug.Log("trying to connect to " + host + ":" + port);
				if (Captury_connect(host, port) == 1 && Captury_synchronizeTime() != 0)
                {
                    isSetup = true;
                    Debug.Log("Successfully opened port to Captury Live");
                    Debug.Log("The time difference is " + Captury_getTimeOffset());
                } else
					Debug.Log(String.Format("Unable to connect to Captury Live at {0}:{1} ", host, port));
            }
            if (isSetup)
            {
                // grab actors
                IntPtr actorData = IntPtr.Zero;
                int numActors = Captury_getActors(out actorData);
                if (numActors > 0 && actorData != IntPtr.Zero)
                {
                    Debug.Log(String.Format("Successfully received {0} actors", numActors));

                    // create actor struct
                    int szStruct = Marshal.SizeOf(typeof(CapturyActor));
                    for (uint i = 0; i < numActors; i++)
                    {
                        // get an actor
                        CapturyActor actor = new CapturyActor();
                        actor = (CapturyActor)Marshal.PtrToStructure(new IntPtr(actorData.ToInt64() + (szStruct * i)), typeof(CapturyActor));
						Debug.Log(String.Format("actor id {0} {1} ", actor.id, Encoding.UTF8.GetString(actor.name)));
						
                        IntPtr ptr = new IntPtr(actorData.ToInt64() + (szStruct * i));
/*						for (int x = 0; x < 64; ++x) {
							Debug.Log(String.Format("byte {0} {1}", x, Marshal.ReadByte(ptr, x)));
						}*/					

                        // check if we already have it in our dictionary
                        if (actors.ContainsKey(actor.id))
                        {
                            actorFound[actor.id] = 5;
                            Debug.Log("Found known actor " + actor.id);
                            continue;
                        }
                        Debug.Log("Found new actor " + actor.id + " with " + actor.numJoints + " joints");

                        // no? we need to convert it
                        IntPtr actorPointer = new IntPtr(actorData.ToInt64() + (szStruct * i));
                        CapturySkeleton skeleton = new CapturySkeleton();
                        ConvertActor(actor, ref skeleton);

                        //  and add it to the list of actors we are processing, making sure this is secured by the mutex
                        communicationMutex.WaitOne();
                        actors.Add(actor.id, actor);
                        actorPointers.Add(actor.id, actorPointer);
                        skeletons.Add(actor.id, skeleton);
                        actorFound.Add(actor.id, 5);
                        communicationMutex.ReleaseMutex();
                    }

                    if (!isConnected && Captury_startStreaming(1) == 1)
                    {
                        Debug.Log("Successfully started streaming data");
                        isConnected = true;
                    }
                }

                // reduce the actor countdown by one for each actor
                int[] keys = new int[actorFound.Keys.Count];
                actorFound.Keys.CopyTo(keys, 0);
                foreach (int key in keys)
                    actorFound[key]--;
            }

            // remove all actors that were not found in the past few actor checks
            //			Debug.Log ("Updating actor structure");
            communicationMutex.WaitOne();
            List<int> unusedKeys = new List<int>();
            foreach (KeyValuePair<int, int> kvp in actorFound)
            {
                if (kvp.Value <= 0)
                {
                    // check if this is the current actor
                    if (kvp.Key == playerActor)
                        playerActor = -1;

                    // remove actor
					Debug.Log("Removing actor " + kvp.Key);
                    actors.Remove(kvp.Key);
                    actorPointers.Remove(kvp.Key);
                    skeletons.Remove(kvp.Key);
                    unusedKeys.Add(kvp.Key);
                }
            }
            communicationMutex.ReleaseMutex();
            //			Debug.Log ("Updating actor structure done");

            // clear out actorfound structure
            foreach (int key in unusedKeys)
                actorFound.Remove(key);

            // look for current transformation of bones with ar tags - the head
            foreach (KeyValuePair<int, IntPtr> kvp in actorPointers)
            {
                int id = kvp.Key;

                // find the index of the head joint
                int headJointIndex = -1;
                for (int i = 0; i < skeletons[id].joints.Length; ++i)
                {
                    if (skeletons[id].joints[i].name.EndsWith(headJointName))
                    {
                        headJointIndex = i;
                        break;
                    }
                }
                if (headJointIndex == -1)
                {
                    Debug.Log("no head joint for skeleton " + id);
                    continue;
                }

                // get the transform and store it in headTransforms
                IntPtr trafo = Marshal.AllocHGlobal(Marshal.SizeOf(typeof(CapturyTransform)));
                UInt64 timestamp = Captury_getMarkerTransform(kvp.Value, headJointIndex, trafo);
                // is there a constraint for this joint that is not older than 500ms?
                if (timestamp != 0)
                {
                    CapturyTransform t = (CapturyTransform)Marshal.PtrToStructure(trafo, typeof(CapturyTransform));
                    communicationMutex.WaitOne();
                    if (headTransforms.ContainsKey(id))
                    {
                        // this is a new transform. the other thread should have a look at it.
                        if (headTransforms[id].timestamp < timestamp)
                            headTransforms[id].consumed = false;
                    }
                    else
                    {
                        headTransforms[id] = new CapturyMarkerTransform();
                        headTransforms[id].bestAccuracy = 0.95f;
                        // if the new transform is actually already old mark it as old directly
                        if (timestamp > Captury_getTime() - 500000)
                            headTransforms[id].consumed = false;
                        else
                            headTransforms[id].consumed = true;
                    }
                    headTransforms[id].rotation = ConvertRotation(new Vector3(t.rx * 180 / (float)Math.PI, t.ry * 180 / (float)Math.PI, t.rz * 180 / (float)Math.PI));
                    headTransforms[id].translation = ConvertPosition(new Vector3(t.tx, t.ty, t.tz));
                    headTransforms[id].timestamp = timestamp;
                    communicationMutex.ReleaseMutex();
//                    Debug.Log(string.Format("transform for actor.joint {0}.{1} is good, really t {2}, delta now {3}", id, headJointIndex, timestamp, Captury_getTime() - timestamp));
                }
                else
                {
                    communicationMutex.WaitOne();
                    headTransforms.Remove(id);
                    communicationMutex.ReleaseMutex();
                }
                Marshal.FreeHGlobal(trafo);
            }
        }

        Debug.Log("Disconnecting");
        // make sure we disconnect
        Captury_disconnect();
        isSetup = false;
        isConnected = false;
    }


    //=============================
    // this is run once at startup
    //=============================
   
    void Start()
    {
        Debug.Log("Conneting to PhotonServer...");
        PhotonNetwork.ConnectUsingSettings("v1.0");
        // start the connection thread
        communicationThread = new Thread(lookForActors);
        communicationThread.Start();

    }

    //==========================
    // this is run once at exit
    //==========================
    void OnDisable()
    {
        communicationFinished = true;
        communicationThread.Join();
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
					Debug.Log("lol: " + actorTemplateObject.name);
					actor.SetActive(true);
					actorObjects.Add(actorID, actor);
                    ConnectSkeleton(actorID, actor.transform);
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

                // check whether we can calibrate the coordinate systems against each other
/*                if (headTransform != null && headTransforms.ContainsKey(actorID) && !headTransforms[actorID].consumed && headTransforms[actorID].timestamp > Captury_getTime() - 500000)
                {
                    Camera cam = GameObject.Find("/OVRController/Camera").GetComponent<Camera>();
                    GameObject cube = GameObject.Find("/Cube");
                    cube.transform.rotation = headTransforms[actorID].rotation;

                    Quaternion delta = headTransforms[actorID].rotation * Quaternion.Inverse(cam.transform.localRotation);
                    float angle;
                    Vector3 axis;
                    cam.transform.localRotation.ToAngleAxis(out angle, out axis);
                    Debug.Log(String.Format("oculus angle {0} axis {1} {2} {3} ", angle, axis.x, axis.y, axis.z));
                    headTransforms[actorID].rotation.ToAngleAxis(out angle, out axis);
                    Debug.Log(String.Format("head angle {0} axis {1} {2} {3} ", angle, axis.x, axis.y, axis.z));
                    delta.ToAngleAxis(out angle, out axis);
                    float dist = (headTransforms[actorID].translation - headTransform.position).magnitude;
                    Debug.Log(String.Format("delta angle {0} axis {1} {2} {3} dist {4}", angle, axis.x, axis.y, axis.z, dist));
                    if (dist < 0.2 && Math.Abs(axis.y) > headTransforms[actorID].bestAccuracy)
                    {
                        CameraFollower cfScript = GameObject.Find("/OVRController").GetComponent<CameraFollower>();
                        cfScript.orientationOffset = Quaternion.Euler(0,180,0) * delta;// Quaternion.Inverse(delta);
                        headTransforms[actorID].bestAccuracy = Math.Abs(axis.y);
                    }
                    headTransforms[actorID].consumed = true;
                }

                // send head orientation back to captury live to refine the tracking
                if (playerActor == actorID && cameraTransform != null)
                {
                    Quaternion q = ConvertRotationToLive(cameraTransform.rotation);
                    float[] array = new float[4];
                    array[0] = q.x;
                    array[1] = q.y;
                    array[2] = q.z;
                    array[3] = q.w;
                    GCHandle handle = GCHandle.Alloc(array, GCHandleType.Pinned);
                    IntPtr data = handle.AddrOfPinnedObject();
                    // TODO: only send constraint back when we have a common calibration between live and the oculus
                    //int result = Captury_setRotationConstraint(actorID, 7, data, 0, 1.0f);
                    if (handle.IsAllocated)
                        handle.Free();
                }
*/
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
                float[] values = new float[pose.numValues*6];
                Marshal.Copy(pose.values, values, 0, pose.numValues*6);
		
		Debug.Log("Value :" + values.Length);

                Debug.Log("received pose with numvalues: " + pose.numValues + " skeleton has joints: " + skeletons[actorID].joints.Length);
		Debug.Log("Send values over photon network...");
		this.photonView.RPC("sendValues",PhotonTargets.Others,values);
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
   public void sendValues(float[] values){
	Debug.Log("received Values with Length: ");
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
    private void ConvertActor(CapturyActor actor, ref CapturySkeleton skel)
    {
        if (skel == null)
        {
            Debug.Log("Null skeleton reference");
            return;
        }

        // copy data over
        skel.name = System.Text.Encoding.UTF8.GetString(actor.name);
        skel.id = actor.id;

        // create joints
        int szStruct = Marshal.SizeOf(typeof(CapturyJoint));
        skel.joints = new CapturySkeletonJoint[actor.numJoints];
        for (uint i = 0; i < actor.numJoints; i++)
        {
            // marshall the joints into a new joint struct
            CapturyJoint joint = new CapturyJoint();
            joint = (CapturyJoint)Marshal.PtrToStructure(new IntPtr(actor.joints.ToInt64() + (szStruct * i)), typeof(CapturyJoint));

            skel.joints[i] = new CapturySkeletonJoint();
            skel.joints[i].name = System.Text.Encoding.ASCII.GetString(joint.name);
            int jpos = skel.joints[i].name.IndexOf("\0");
            skel.joints[i].name = skel.joints[i].name.Substring(0, jpos);
            skel.joints[i].offset.Set(joint.ox, joint.oy, joint.oz);
            skel.joints[i].orientation.Set(joint.rx, joint.ry, joint.rz);

            //Debug.Log ("Got joint " + skel.joints[i].name + " at " + joint.ox + joint.oy + joint.oz);
        }
    }


    //=========================================
    // connect a given transform to a skeleton
    //=========================================
    private void ConnectSkeleton(int actorId, Transform transform)
    {
        if (transform == null || !actors.ContainsKey(actorId))
            return;
			
        communicationMutex.WaitOne();
        CapturyActor actor = actors[actorId];
        CapturySkeleton skel = skeletons[actorId];
        skel.reference = transform;

        for (uint i = 0; i < actor.numJoints; i++)
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
