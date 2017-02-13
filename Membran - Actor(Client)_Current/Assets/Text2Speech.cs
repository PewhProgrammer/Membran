using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Text2Speech : Photon.PunBehaviour {
    
    private string result = "";
    private bool stop = false;
    public GameObject meshNode;

	// Use this for initialization
	void Start () {
        //PhotonNetwork.ConnectUsingSettings("v1.0");
    }

    [PunRPC]
    public void sendString(string result)
    {
        if (!result.Equals("")) {
            this.result = result;
        }

        bool stopIndicator = false;
        if (result.ToLower().Equals("stopp")) {
            stop = !stop;
            meshNode.GetComponent<Fading>().toggleFading();
            stopIndicator = true;
        }

        if (!stop && !stopIndicator && !result.Equals("")) {
            WindowsVoice.theVoice.speak(result);
        }
    }

    override
    public void OnConnectedToMaster()
    {
        RoomOptions rOpts = new RoomOptions() { IsVisible = false, MaxPlayers = 5 };
        PhotonNetwork.JoinOrCreateRoom("Actor", rOpts, TypedLobby.Default);
    }
	
	// Update is called once per frame
	void Update () {
		
	}

    void OnGUI()
    {
        GUI.Label(new Rect(20, 20, 1000, 20), PhotonNetwork.connectionStateDetailed.ToString());
        GUI.Label(new Rect(20, 40, 1000, 20), "Voice output activated: " + (!stop).ToString());
        GUI.Label(new Rect(20, 60, 1000, 20), "Uni voice input: " + result);
    }
}
